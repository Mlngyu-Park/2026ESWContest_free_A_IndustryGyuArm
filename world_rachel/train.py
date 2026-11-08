# 월드 레이첼 학습기 — Spot 이 물리 속에서 서고 걷는 법을 PPO(시행착오 강화학습)로 배운다.
#   연습장은 Node 일꾼들(worker.mjs, 앱과 같은 Rapier 물리), 신경망은 여기(torch, GPU).
#   python train.py --workers 10 --envs 128 --iters 1500 --out runs/v1
#   스파크: taskset -c 0-9 로 띄운다 (CPU 10개 제한).
import argparse, json, os, struct, subprocess, sys, time
import numpy as np
import torch, torch.nn as nn

ACT = 12
HERE = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser()
ap.add_argument("--workers", type=int, default=10)
ap.add_argument("--envs", type=int, default=128)        # 일꾼 하나당 연습장 수
ap.add_argument("--iters", type=int, default=1500)
ap.add_argument("--steps", type=int, default=24)         # 한 번 모으는 걸음 수 (연습장마다)
ap.add_argument("--out", default="runs/v1")
ap.add_argument("--node", default="node")
ap.add_argument("--resume", default="")
ap.add_argument("--save_every", type=int, default=50)
ap.add_argument("--ver", type=int, default=1)          # 1 = 걷기만 · 2 = + 몸 높이·숙임, 팔 무게 · 3 = 2 + 엄격한 따라가기 · 4 = 3 + 쌓인 오차·잘게 맞추기 · 5 = 4 + 숙인 채 걷기 · 6 = 5 + 발 말고 닿으면 감점
args = ap.parse_args()
OBS = {1: 51, 2: 53, 3: 53, 4: 53, 5: 53, 6: 53}[args.ver]
os.makedirs(args.out, exist_ok=True)
dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
LOG = open(os.path.join(args.out, "train.log"), "a", buffering=1)
def log(s):
    line = time.strftime("%H:%M:%S ") + s
    print(line, flush=True); LOG.write(line + "\n")

# ── 일꾼 ──
class Workers:
    def __init__(self, n, envs):
        self.n, self.envs = n, envs
        self.p = [subprocess.Popen([args.node, os.path.join(HERE, "worker.mjs"), str(envs), str(i + 1 + int(time.time()) % 10000), str(args.ver)],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, cwd=HERE, bufsize=0) for i in range(n)]
    def _send(self, i, cmd, payload=b""):
        self.p[i].stdin.write(struct.pack("<ii", cmd, len(payload)) + payload)
    def _read(self, i, nbytes):
        buf = bytearray(nbytes); mv = memoryview(buf); got = 0
        while got < nbytes:
            k = self.p[i].stdout.readinto(mv[got:])
            if not k: raise RuntimeError(f"일꾼 {i} 이 멈췄습니다")
            got += k
        return buf
    def reset(self):
        for i in range(self.n): self._send(i, 1)
        return np.concatenate([np.frombuffer(self._read(i, self.envs * OBS * 4), np.float32).reshape(self.envs, OBS) for i in range(self.n)])
    def step(self, act):
        a = act.reshape(self.n, self.envs * ACT).astype(np.float32)
        for i in range(self.n): self._send(i, 2, a[i].tobytes())
        o, r, d, t = [], [], [], []
        E = self.envs
        for i in range(self.n):
            x = np.frombuffer(self._read(i, E * (OBS + 3) * 4), np.float32)
            o.append(x[:E * OBS].reshape(E, OBS)); r.append(x[E * OBS:E * OBS + E]); d.append(x[E * OBS + E:E * OBS + 2 * E]); t.append(x[E * OBS + 2 * E:])
        return np.concatenate(o), np.concatenate(r), np.concatenate(d), np.concatenate(t)
    def stats(self):
        for i in range(self.n): self._send(i, 3)
        out = []
        for i in range(self.n):
            (k,) = struct.unpack("<i", self._read(i, 4)); out += json.loads(bytes(self._read(i, k)))
        return out
    def close(self):
        for i in range(self.n):
            try: self._send(i, 9)
            except Exception: pass

# ── 신경망 ──
def mlp(sizes):
    L = []
    for a, b in zip(sizes[:-1], sizes[1:]): L += [nn.Linear(a, b), nn.ELU()]
    L = L[:-1]
    return nn.Sequential(*L)

class Norm(nn.Module):                                   # 관찰 평균·분산 (앱에서도 같이 쓴다)
    def __init__(self, n):
        super().__init__()
        self.register_buffer("mean", torch.zeros(n)); self.register_buffer("var", torch.ones(n)); self.register_buffer("count", torch.tensor(1e-4))
    def update(self, x):
        bm, bv, bc = x.mean(0), x.var(0, unbiased=False), x.shape[0]
        d = bm - self.mean; tot = self.count + bc
        self.mean += d * bc / tot
        self.var = (self.var * self.count + bv * bc + d * d * self.count * bc / tot) / tot
        self.count = tot
    def forward(self, x): return torch.clamp((x - self.mean) / torch.sqrt(self.var + 1e-8), -5, 5)

class AC(nn.Module):
    def __init__(self):
        super().__init__()
        self.norm = Norm(OBS)
        self.actor = mlp([OBS, 256, 128, 64, ACT])
        self.critic = mlp([OBS, 512, 256, 128, 1])
        self.log_std = nn.Parameter(torch.zeros(ACT))
    def dist(self, o):
        mu = self.actor(o); return torch.distributions.Normal(mu, self.log_std.exp().expand_as(mu))
    def value(self, o): return self.critic(o).squeeze(-1)

net = AC().to(dev)
opt = torch.optim.Adam(net.parameters(), lr=1e-3)
it0 = 0
if args.resume:
    ck = torch.load(args.resume, map_location=dev); net.load_state_dict(ck["net"]); opt.load_state_dict(ck["opt"]); it0 = ck["it"]
    log(f"이어서: {args.resume} ({it0}번째부터)")

def export(path):                                        # 앱(JS)에서 쓰는 꼴: 정규화 + 행동 신경망만
    L = [m for m in net.actor if isinstance(m, nn.Linear)]
    js = {"ver": args.ver, "obs": OBS, "act": ACT, "mean": net.norm.mean.tolist(), "std": torch.sqrt(net.norm.var + 1e-8).tolist(), "clip": 5.0, "activation": "elu",
          "layers": [{"W": m.weight.detach().cpu().numpy().round(6).tolist(), "b": m.bias.detach().cpu().numpy().round(6).tolist()} for m in L]}
    with open(path, "w") as f: json.dump(js, f)

# ── PPO ──
GAMMA, LAM, CLIP, EPOCHS, MINI, ENT, VCOEF, KL_T = 0.99, 0.95, 0.2, 5, 4, 0.005, 1.0, 0.01
W = Workers(args.workers, args.envs); NE = args.workers * args.envs; T = args.steps
log(f"시작: 일꾼 {args.workers} × 연습장 {args.envs} = {NE}, 걸음 {T}, 장치 {dev}")
obs = torch.tensor(W.reset(), device=dev)
lr = 1e-3
try:
    for it in range(it0, it0 + args.iters):
        t0 = time.time()
        B_o = torch.zeros(T, NE, OBS, device=dev); B_a = torch.zeros(T, NE, ACT, device=dev); B_lp = torch.zeros(T, NE, device=dev)
        B_r = torch.zeros(T, NE, device=dev); B_d = torch.zeros(T, NE, device=dev); B_v = torch.zeros(T, NE, device=dev); B_mu = torch.zeros(T, NE, ACT, device=dev)
        with torch.no_grad():
            for t in range(T):
                net.norm.update(obs)
                on = net.norm(obs); d = net.dist(on); a = d.sample()
                B_o[t] = obs; B_a[t] = a; B_lp[t] = d.log_prob(a).sum(-1); B_v[t] = net.value(on); B_mu[t] = d.mean
                o2, r, dn, to = W.step(a.cpu().numpy())
                r = torch.tensor(r, device=dev); dn = torch.tensor(dn, device=dev); to = torch.tensor(to, device=dev)
                r = r + GAMMA * B_v[t] * to                  # 시간이 다 돼 끝난 판은 '앞으로 받을 점수' 를 이어 붙인다
                B_r[t] = r; B_d[t] = dn; obs = torch.tensor(o2, device=dev)
            last_v = net.value(net.norm(obs))
            adv = torch.zeros_like(B_r); g = torch.zeros(NE, device=dev)
            for t in reversed(range(T)):
                nv = last_v if t == T - 1 else B_v[t + 1]
                delta = B_r[t] + GAMMA * nv * (1 - B_d[t]) - B_v[t]
                g = delta + GAMMA * LAM * (1 - B_d[t]) * g; adv[t] = g
            ret = adv + B_v
        t_col = time.time() - t0
        std_old = net.log_std.exp().detach().clone()
        # 배우기
        fo, fa, flp, fadv, fret, fv, fmu = B_o.reshape(-1, OBS), B_a.reshape(-1, ACT), B_lp.reshape(-1), adv.reshape(-1), ret.reshape(-1), B_v.reshape(-1), B_mu.reshape(-1, ACT)
        fadv = (fadv - fadv.mean()) / (fadv.std() + 1e-8)
        N = fo.shape[0]; mb = N // MINI; kls = []
        for ep in range(EPOCHS):
            perm = torch.randperm(N, device=dev)
            for k in range(MINI):
                idx = perm[k * mb:(k + 1) * mb]
                on = net.norm(fo[idx]); d = net.dist(on); lp = d.log_prob(fa[idx]).sum(-1)
                ratio = torch.exp(lp - flp[idx])
                pl = -torch.min(ratio * fadv[idx], torch.clamp(ratio, 1 - CLIP, 1 + CLIP) * fadv[idx]).mean()
                v = net.value(on); vc = fv[idx] + torch.clamp(v - fv[idx], -CLIP, CLIP)
                vl = torch.max((v - fret[idx]) ** 2, (vc - fret[idx]) ** 2).mean()
                ent = d.entropy().sum(-1).mean()
                loss = pl + VCOEF * vl - ENT * ent
                opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(net.parameters(), 1.0); opt.step()
                with torch.no_grad():
                    sn = d.stddev                            # KL: 옛 정책과 얼마나 달라졌나 → 학습률 조절 (정규분포 정확식)
                    kl = torch.sum(torch.log(sn / std_old) + (std_old ** 2 + (fmu[idx] - d.mean) ** 2) / (2 * sn ** 2) - 0.5, -1).mean().item(); kls.append(kl)
                if kl > KL_T * 2: lr = max(1e-5, lr / 1.5)
                elif kl < KL_T / 2: lr = min(1e-2, lr * 1.5)
                for gp in opt.param_groups: gp["lr"] = lr
        dt = time.time() - t0
        st = W.stats()
        if st:
            ret_m = np.mean([e["ret"] for e in st]); len_m = np.mean([e["len"] for e in st]); fall = np.mean([e["fell"] for e in st])
            terms = {k: np.mean([e["terms"].get(k, 0) for e in st]) for k in st[0]["terms"]}
            ts = " ".join(f"{k}={v:.3f}" for k, v in terms.items())
        else: ret_m = len_m = fall = float("nan"); ts = ""
        log(f"[{it+1}] 걸음/초 {N/dt:.0f} (모으기 {t_col:.1f}s/전체 {dt:.1f}s) 판 {len(st)} 점수 {ret_m:.2f} 길이 {len_m:.1f}s 넘어짐 {fall*100:.0f}% "
            f"흔들림 {net.log_std.exp().mean().item():.2f} 학습률 {lr:.1e} KL {np.mean(kls):.4f} | {ts}")
        if (it + 1) % args.save_every == 0 or it + 1 == it0 + args.iters:
            torch.save({"net": net.state_dict(), "opt": opt.state_dict(), "it": it + 1}, os.path.join(args.out, f"ck_{it+1:05d}.pt"))
            export(os.path.join(args.out, f"policy_{it+1:05d}.json")); export(os.path.join(args.out, "policy_latest.json"))
finally:
    W.close()
