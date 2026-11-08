// 월드 레이첼 두뇌 (학습된 행동 신경망) — 관찰 51개 → 관절 목표 12개. 브라우저·Node 같은 코드, 바깥 라이브러리 없음.
//   train.py 의 export() 가 만든 JSON 을 그대로 읽는다: 정규화(평균·표준편차) → 층마다 W·x + b → ELU (마지막 층 빼고)
export class Policy {
  constructor(js) {
    this.mean = Float32Array.from(js.mean); this.std = Float32Array.from(js.std); this.clip = js.clip ?? 5;
    this.layers = js.layers.map(L => ({ W: Float32Array.from(L.W.flat()), b: Float32Array.from(L.b), nin: L.W[0].length, nout: L.b.length }));
    const widest = Math.max(js.obs, ...this.layers.map(L => L.nout));
    this.bufA = new Float32Array(widest); this.bufB = new Float32Array(widest);
  }
  act(obs, out = new Float32Array(this.layers.at(-1).nout)) {
    let x = this.bufA, y = this.bufB;
    for (let i = 0; i < obs.length; i++) x[i] = Math.max(-this.clip, Math.min(this.clip, (obs[i] - this.mean[i]) / this.std[i]));
    this.layers.forEach((L, li) => {
      const last = li === this.layers.length - 1, dst = last ? out : y;
      for (let o = 0; o < L.nout; o++) {
        let s = L.b[o]; const row = o * L.nin;
        for (let i = 0; i < L.nin; i++) s += L.W[row + i] * x[i];
        dst[o] = last ? s : (s > 0 ? s : Math.expm1(s));
      }
      if (!last) { const t = x; x = y; y = t; }
    });
    return out;
  }
}
