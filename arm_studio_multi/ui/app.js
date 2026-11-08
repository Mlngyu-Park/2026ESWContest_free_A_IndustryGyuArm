/* ================= ARM STUDIO 다중 로봇 — frontend (11-02) =================
   ARM STUDIO 프로 화면의 app.js 를 바탕으로: 콘솔·전광판·API·탭·블록 팔레트·파이프라인(시작/끝 캡,
   이중 블록 [로봇][동작][사물 슬롯])·끌어 놓기·쓰레기통·타임바·블록 메뉴는 원본 방식 그대로.
   서버(server.mjs)가 G1 · Spot · UR15 계산과 책 물리를 돌린다. */
'use strict';
const $=s=>document.querySelector(s), $$=s=>[...document.querySelectorAll(s)];

let META=null;          // /api/meta
let ST=null;            // 마지막 /api/state
let sel=-1;             // 파이프라인 선택 블록 번호
const HASH={};          // 섹션별 변경 감지
let lastLogN=0;

/* ---------- 콘솔 + 전광판 (원본) ---------- */
function log(msg,err){
  if(!msg)return;
  const c=$('#console'),d=document.createElement('div');
  const t=new Date().toTimeString().slice(0,8);
  d.innerHTML=`<span class="t">${t}</span>`+(err?`<span class="err"></span>`:'');
  (err?d.querySelector('.err'):d).append(msg);
  c.append(d);c.scrollTop=c.scrollHeight;
  while(c.children.length>300)c.firstChild.remove();
  setTicker(String(msg));
}
function setTicker(t){
  const s=$('#tickerText');
  if(s.textContent===t)return;          // 같은 내용이면 애니메이션 유지
  s.textContent=t;
  s.style.animation='none';void s.offsetWidth;s.style.animation='';
}

/* ---------- API (원본과 같은 /api/call) ---------- */
async function call(fn,...args){
  try{
    const r=await fetch('/api/call',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({fn,args})});
    const j=await r.json();
    if(!j.ok){log(`${j.msg||fn+' 실패'}`,true);}
    await refresh();
    return j;
  }catch(e){log(`${fn}: ${e}`,true);return null;}
}
async function refresh(){
  try{ST=await(await fetch('/api/state')).json();render(ST);}catch(e){}
}
function changed(key,data){
  const j=JSON.stringify(data);
  if(HASH[key]===j)return false;HASH[key]=j;return true;
}
function txtColor(hex){
  try{
    const n=parseInt(String(hex).slice(1),16);
    const r=n>>16&255,g=n>>8&255,b=n&255;
    return (0.299*r+0.587*g+0.114*b)>150?'#111':'#fff';
  }catch(e){return '#fff';}
}
function esc(s){return String(s).replace(/[&<>"]/g,
  c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));}
function segBtns(el,labels,cur,cb){
  el.innerHTML='';
  labels.forEach(l=>{
    const b=document.createElement('button');
    b.textContent=l;if(l===cur)b.classList.add('on');
    b.onclick=()=>cb(l);el.append(b);
  });
}

/* ---------- 블록 모양: 동작 색 (원본의 이동=파랑 · 잡기=초록 · 놓기=빨강 · 그리퍼/기다리기=주황) ---------- */
const BLK_CLS={move:'pose',floor:'brick',wait:'grip',
  walk:'pose',turn:'pose',grab:'brick',place:'brick rel',body:'grip',wave:'grip',rest:'grip'};
const ARG_CLS={cell:'brick rel',item:'brick',place:'brick rel',target:'pose'};
const ROBOT_TAG={ur15:'UR15',spot:'Spot',g1:'G1'};
/* 건드린 책(케어)을 언제 고치나 — 고른 것마다 한 줄 풀이 */
const FIX_HINT={
  now:'누가 책을 옮기면 하던 일을 멈추고 그 책부터 제자리로 (손에 든 책은 먼저 꽂음)',
  book:'지금 옮기는 책을 마저 꽂은 뒤, 건드린 책을 제자리로',
  block:'지금 블록을 다 끝낸 뒤, 건드린 책을 제자리로',
  end:'블록을 모두 끝낸 뒤 마무리 단계에서 한꺼번에',
};
/* 블록 나무 접기: 키 → 펼침(true)/접음(false). 없으면 기본 — 블록은 펼침, 책은 지금 하는 책만 펼침 */
const FOLD={};
const isOpen=(key,dflt)=>FOLD[key]??dflt;
function toggle(key,dflt){FOLD[key]=!isOpen(key,dflt);HASH.seq=null;render(ST);}
const stCls=n=>n.state==='cur'?' on':n.state==='done'?' done':n.pending?' wait':'';
const nodeTitle=n=>n.skip?'고치기로 이미 목표 칸에 갔거나 사용자가 미리 넣어 줘서 건너뜀'
  :n.pending?'건드린 책 — 위 "건드린 책" 에서 고른 차례에 여기서 고칩니다'
  :n.fix?'건드린 책을 ▶ 때 정한 목표 칸으로 고치기':'';
/* 책 한 권 → 단계(이동 · 잡기 · 놓기) → 칸(열기 · 넣기 …). 원본 잡기 블록의 세부 단계 칸과 같은 모양 */
function treeEl(nodes){
  const box=document.createElement('div');box.className='tree';
  for(const n of nodes){
    const key='n'+n.id, open=isOpen(key,n.state==='cur');
    const r=document.createElement('div');
    r.className='tnode'+(n.state?' '+n.state:'')+(n.fix?' fix':'')+(n.pending?' wait':'');
    r.innerHTML=`<span class="fold">${open?'▾':'▸'}</span><span class="objDot" style="background:${esc(n.color)}"></span>`
      +`<span class="tname">${n.fix?'⟲ 고치기 ':''}${esc(n.book)}</span><span class="tsub">${esc(n.from)} → ${esc(n.to)}</span>`
      +`<span class="tstate">${n.state==='done'?(n.skip?'건너뜀':'✓'):n.state==='cur'?(n.note?esc(n.note):'▶ 지금'):n.pending?'고치기 대기'+(n.why?' · '+esc(n.why):''):''}</span>`;
    r.title=nodeTitle(n);
    r.onclick=e=>{e.stopPropagation();toggle(key,n.state==='cur');};
    box.append(r);
    if(!open)continue;
    for(const l of n.leaves){
      const lf=document.createElement('div');lf.className='tleaf'+(l.state?' '+l.state:'');
      const nm=document.createElement('span');nm.className='lname';nm.textContent=l.label;lf.append(nm);
      const ss=document.createElement('span');ss.className='subSteps';
      const ci=l.chip?l.chips.indexOf(l.chip):-1;
      l.chips.forEach((c,j)=>{const b=document.createElement('span');
        b.className='subStep'+(l.state==='done'||(l.state==='cur'&&j<ci)?' done':l.state==='cur'&&j===ci?' on':'');
        b.textContent=c;ss.append(b);});
      lf.append(ss);box.append(lf);
    }
  }
  return box;
}

/* ---------- 렌더 ---------- */
function render(s){
  if(!META)return;
  /* 상단 칩 */
  $('#robotChip').textContent=`${META.robots[s.robot]} · ${META.scenes.find(x=>x.id===s.scene)?.name||''}`;
  $('#robotChip').classList.add('on');
  /* 전광판: 서버 기록에서 새 줄만 */
  for(const x of (s.log||[]))if(x.n>lastLogN){log(`${x.t.toFixed(1)}초  ${x.msg}`,x.cls==='bad');lastLogN=x.n;}
  $('#ticker').classList.toggle('busy',!!s.run.running);
  $('#btnPreview').disabled=!!s.run.running;

  /* ① 공장 꾸미기 */
  if(changed('scenes',[s.scene,s.robot])){
    const sc=$('#sceneSel');
    sc.innerHTML=META.scenes.map(x=>`<option value="${x.id}"${x.id===s.scene?' selected':''}>${esc(x.name)}</option>`).join('');
    const cur=META.scenes.find(x=>x.id===s.scene);
    $('#robotSel').innerHTML=[...cur.robots.map(r=>`<option value="${r}"${r===s.robot?' selected':''}>${esc(META.robots[r])}</option>`),
      ...cur.soon.map(r=>`<option value="${r}" disabled>${esc(META.robots[r])} (준비 중)</option>`)].join('');
    $('#envHint').textContent=s.scene==='books'?'돈케어 — 3D 에서 끌어 옮기면 로봇이 바뀐 자리로 찾아갑니다 (끄는 동안 Q/E · 휠 = 돌리기)':'이 장면의 설비 (자리는 고정)';
    $('#objHint').textContent=s.scene==='books'?'케어 — 지금 칸 → 목표 칸. ▶ 를 누를 때 목표를 정하고, 도중에 누가 옮겨도 로봇이 목표로 고쳐 놓습니다':'지금 어디에 있는지';
    $('#robotNote').textContent=s.scene==='books'
      ?'UR15 는 가운데 받침대에 고정, Spot 은 칸 앞으로 걸어갑니다. G1 은 아직 이 장면에 없습니다.'
      :'G1 은 두 발로, Spot 은 네 발로 걸어가 잡고 옮깁니다.';
  }
  /* 설비 = 돈케어 (어디로 옮기든 로봇이 칸 이름으로 찾아감) · 물건(책) = 케어 (▶ 때 정한 목표 칸을 로봇이 지킴) */
  if(changed('env',s.fixtures)){
    $('#envList').innerHTML=(s.fixtures||[]).map(e=>`<button disabled title="${e.care===false?(e.fixed?'고정 설비':'돈케어: 3D 에서 끌어 옮겨도 로봇이 칸 이름으로 찾아갑니다'):''}">${esc(e.name)}${e.care===false&&!e.fixed?' <span class="careTag dont">돈케어</span>':''}</button>`).join('');
  }
  if(changed('objs',[s.objects,s.goalLive])){
    $('#objList').innerHTML=s.objects.map(o=>`<div class="objRow"><span class="objDot" style="background:${o.color}"></span>${esc(o.name)}`
      +(o.care?' <span class="careTag">케어</span>':'')
      +`<span class="objWhere">${esc(o.where)}${o.goal&&o.goal!==o.where?` → <b class="goal${s.goalLive?' live':''}">${esc(o.goal)}</b>`:o.goal?' <span class="ok">✓</span>':''}</span></div>`).join('');
  }
  if(s.drag) setTicker(`✋ ${s.drag.name} 옮기는 중 — 로봇 멈춤`+(s.drag.ok?'':` · ${s.drag.why} (놓으면 제자리로)`));

  /* Spot 걷기: 계산 · AI (책꽂이 장면의 Spot 만) */
  if(changed('walk',[s.scene,s.robot,s.walk,s.walkBrain,s.fell])){
    $('#walkBox').style.display=s.scene==='books'&&s.robot==='spot'&&s.walks?'':'none';
    if(s.walks){
      const keys=Object.keys(s.walks).filter(k=>k!=='ai'||s.walkBrain);
      segBtns($('#walkSeg'),keys.map(k=>s.walks[k]),s.walks[s.walk],l=>call('walk_mode',keys.find(k=>s.walks[k]===l)));
      $('#walkHint').textContent=s.fell?'Spot 이 넘어졌습니다 — ⌂ 처음 상태로 다시 세우세요'
        :s.walk==='ai'?`다리 12개를 물리로 움직이고, 학습한 두뇌(월드 레이첼 ${s.walkBrain})가 균형을 잡습니다. 팔·잡기는 그대로 계산`
        :s.walkBrain?'발 자리를 계산해 그대로 놓습니다 (물리 없음)':'발 자리를 계산해 그대로 놓습니다 — AI 걷기는 학습된 두뇌가 생기면 고를 수 있습니다';
    }
  }
  /* ② 블록 팔레트 */
  renderPalette(s);
  /* 건드린 책을 언제 고치나 — 파이프라인 맨 위 (책꽂이 장면만) */
  if(changed('fix',[s.scene,s.fixMode])){
    $('#fixBox').style.display=s.scene==='books'?'':'none';
    if(s.fixModes){
      const keys=Object.keys(s.fixModes);
      segBtns($('#fixSeg'),keys.map(k=>s.fixModes[k]),s.fixModes[s.fixMode],l=>call('fix_mode',keys.find(k=>s.fixModes[k]===l)));
      $('#fixHint').textContent=FIX_HINT[s.fixMode]||'';
    }
  }
  /* 파이프라인 (스크래치식 이중 블록: [로봇][동작][사물 슬롯]) + 블록 나무 (▸ 펼치면 책마다 단계) */
  if(changed('seq',[s.seq,sel,s.run.running,s.run.blk,s.run.err,s.robot,s.tree])){
    const el=$('#pipeline');el.innerHTML='';
    const cap=(cls,txt)=>{const c=document.createElement('div');
      c.className='blk cap '+cls;c.textContent=txt;el.append(c);};
    cap('start','🚩 시작');
    if(!s.seq.length){
      const h=document.createElement('div');h.className='hint';
      h.style.padding='.4rem 0';
      h.textContent='팔레트에서 블록을 끌어다 여기 꽂으세요.';
      el.append(h);
    }
    s.seq.forEach((it,i)=>{
      const B=META.blocks[s.scene][it.type]||{name:it.type,args:[]};
      const nodes=s.tree?.blocks?.[i]?.nodes||[];
      const d=document.createElement('div');
      d.className='blk '+(BLK_CLS[it.type]||'grip');
      if(i===sel)d.classList.add('on');
      if(s.run.running&&s.run.blk===i)d.classList.add('running');
      d.draggable=true;
      d.innerHTML=`<span class="rbadge" title="이 블록을 하는 로봇">${esc(ROBOT_TAG[s.robot])}</span>
        <span class="act">${esc(B.name)}</span>`;
      B.args.forEach((k,j)=>{
        if(it.type==='move'&&j===1){const a=document.createElement('span');a.className='act';a.textContent='→';d.append(a);}
        const slot=document.createElement('span');
        slot.className='slot '+(ARG_CLS[k]||'');slot.textContent=it.argNames[j]??'◯';
        slot.title='클릭 = 바꾸기 · 팔레트의 사물을 끌어다 끼워도 됩니다';
        slot.onclick=e=>{e.stopPropagation();slotMenu(e,i,j,k);};
        slot.ondragover=e=>{if([...e.dataTransfer.types].includes('k-'+kindOf(k))){e.preventDefault();slot.classList.add('hot');}};
        slot.ondragleave=()=>slot.classList.remove('hot');
        slot.ondrop=async e=>{e.preventDefault();e.stopPropagation();slot.classList.remove('hot');
          const a=JSON.parse(e.dataTransfer.getData('arg')||'null');if(a)await call('seq_set',i,j,a.v);};
        d.append(slot);
      });
      if(s.run.err&&!s.run.running&&s.run.blk===i){
        const eb=document.createElement('span');eb.className='errBadge';eb.textContent='!';eb.title=s.run.err;d.append(eb);}
      d.onclick=()=>{sel=(sel===i)?-1:i;HASH.seq=null;render(ST);};
      d.oncontextmenu=e=>{e.preventDefault();blkMenu(e,i,it);};
      d.ondragstart=e=>{e.dataTransfer.setData('pipe',String(i));e.dataTransfer.setData('text/plain','');e.dataTransfer.effectAllowed='move';};
      if(nodes.length){
        const open=isOpen('b'+i,true);
        const f=document.createElement('span');f.className='fold';f.textContent=open?'▾':'▸';
        f.title=open?'접기':'펼치기 — 책마다 단계';
        f.onclick=e=>{e.stopPropagation();toggle('b'+i,true);};
        d.prepend(f);
        /* 이 블록이 어떤 책들로 이루어졌는지 (오른쪽 작은 칸) — 지금 하는 책 주황, 끝난 책 파랑 */
        const ss=document.createElement('span');ss.className='subSteps';
        nodes.forEach(n=>{const b=document.createElement('span');b.className='subStep'+stCls(n);
          b.textContent=(n.fix?'⟲ ':'')+n.book;b.title=nodeTitle(n);ss.append(b);});
        d.append(ss);
      }
      el.append(d);
      if(nodes.length&&isOpen('b'+i,true))el.append(treeEl(nodes));
    });
    /* 마무리 단계: 블록을 다 하고 나서 제자리가 아닌 책 고치기 ('맨 끝에' 를 고르면 여기서) */
    const E=s.tree?.blocks?.find(b=>b.end);
    if(E&&E.nodes.length){
      const m=document.createElement('div');m.className='tend'+(E.cur?' cur':'');
      m.innerHTML=`<span class="fold">${isOpen('end',true)?'▾':'▸'}</span>🔧 마무리 — 건드린 책 고치기`;
      m.onclick=()=>toggle('end',true);el.append(m);
      if(isOpen('end',true))el.append(treeEl(E.nodes));
    }
    cap('end','🏁 끝');
  }
  renderTimebar(s);
  /* 3D 미리 보기: 반투명 책 = 블록을 다 하면 설 자리 · 흐린 점선 = Spot 이 블록대로 걸을 길 (책꽂이 장면만) */
  if(changed('plan',[s.scene,s.showPlan,s.robot,s.planLegs,s.planMiss])){
    $('#planBox').style.display=s.scene==='books'?'':'none';
    $('#chkPlan').checked=!!s.showPlan;
    $('#planHint').textContent=!s.showPlan?'꺼짐':
      '반투명 책 = 블록을 다 하면 책이 설 자리'+(s.robot==='spot'
        ?` · 흐린 점선 = Spot 이 블록대로 걸을 길 (다리 ${s.planLegs}개`+(s.planMiss?`, 그중 ${s.planMiss}개는 지금 갈 길이 없음`:'')+')'
        :' · UR15 는 받침대에 서서 하므로 걷는 길이 없습니다');
  }
  if(changed('speed',s.speed))segBtns($('#speedSeg'),META.speeds,`${s.speed.toFixed(1)}x`,l=>call('speed',parseFloat(l)));
}
const kindOf=k=>k==='place'||k==='target'?'place':k;   // 끼울 수 있는 사물 종류

/* 슬롯 클릭 = 고르기 메뉴 (원본의 blkMenu 모양) */
function slotMenu(e,i,j,kind){
  document.querySelectorAll('.blkMenu').forEach(x=>x.remove());
  const mn=document.createElement('div');mn.className='blkMenu';
  const head=document.createElement('div');head.className='mhead';head.textContent='바꾸기';mn.append(head);
  for(const [v,t] of (ST.options[kind]||[])){
    const b=document.createElement('button');b.textContent=t;
    b.onclick=async ev=>{ev.stopPropagation();mn.remove();await call('seq_set',i,j,v);};
    mn.append(b);
  }
  mn.style.maxHeight='60vh';mn.style.overflow='auto';
  document.body.append(mn);
  const r=mn.getBoundingClientRect();
  mn.style.left=Math.min(e.clientX,innerWidth-r.width-8)+'px';
  mn.style.top=Math.min(e.clientY,innerHeight-r.height-8)+'px';
  const close=ev=>{if(!mn.contains(ev.target)){mn.remove();document.removeEventListener('mousedown',close);}};
  setTimeout(()=>document.addEventListener('mousedown',close),0);
}
/* 블록 우클릭 메뉴 */
function blkMenu(e,i,it){
  document.querySelectorAll('.blkMenu').forEach(x=>x.remove());
  const mn=document.createElement('div');mn.className='blkMenu';
  const head=document.createElement('div');head.className='mhead';head.textContent=it.text;mn.append(head);
  const add=(txt,fn,hint)=>{const b=document.createElement('button');
    b.innerHTML=txt+(hint?`<small>${esc(hint)}</small>`:'');
    b.onclick=async ev=>{ev.stopPropagation();mn.remove();await fn();};mn.append(b);};
  add('⧉ 복제',async()=>{await call('seq_insert',i+1,JSON.stringify({type:it.type,args:it.args}));});
  add('🗑 이 블록 삭제',async()=>{sel=-1;await call('seq_del',i);});
  document.body.append(mn);
  const r=mn.getBoundingClientRect();
  mn.style.left=Math.min(e.clientX,innerWidth-r.width-8)+'px';
  mn.style.top=Math.min(e.clientY,innerHeight-r.height-8)+'px';
  const close=ev=>{if(!mn.contains(ev.target)){mn.remove();document.removeEventListener('mousedown',close);}};
  setTimeout(()=>document.addEventListener('mousedown',close),0);
}

/* ---------- 블록 팔레트 (원본: 동작 블록 칩 + 사물 알약) ---------- */
function renderPalette(s){
  const el=$('#blkPalette');
  if(!changed('palette',[s.scene,s.objects.map(o=>o.where),s.options]))return;
  el.innerHTML='';
  const cat=t=>{const h=document.createElement('div');h.className='palCat';h.textContent=t;el.append(h);};
  const act=(cls,label,blk,hint)=>{
    const c=document.createElement('div');
    c.className='blkChip '+cls;c.textContent=label;c.title=hint||'끌어서 파이프라인에 놓기';
    c.draggable=true;
    c.ondragstart=e=>{e.dataTransfer.setData('blk',JSON.stringify(blk));e.dataTransfer.setData('text/plain','');e.dataTransfer.effectAllowed='copy';};
    c.ondblclick=()=>call('seq_insert',s.seq.length,JSON.stringify(blk));     // 두 번 누르면 끝에 붙인다
    el.append(c);
  };
  const arg=(kind,label,val,cls,color)=>{
    const c=document.createElement('div');
    c.className='argChip '+cls;c.textContent=label;
    if(color){c.style.background=color;c.style.color=txtColor(color);}
    c.draggable=true;c.title='동작 블록의 구멍(슬롯)에 끼우기';
    c.ondragstart=e=>{e.dataTransfer.setData('arg',JSON.stringify({k:kind,v:val}));e.dataTransfer.setData('k-'+kind,'1');e.dataTransfer.setData('text/plain','');e.dataTransfer.effectAllowed='copy';};
    el.append(c);
  };
  cat('동작 블록 — 파이프라인에 놓기 (두 번 누르면 끝에 붙음)');
  for(const [k,B] of Object.entries(META.blocks[s.scene])){
    const label=B.name+(B.args.length?' '+B.args.map(()=> '◯').join(k==='move'?' → ':' '):'');
    act(BLK_CLS[k]||'grip',label,{type:k,args:[]},B.hint);
  }
  if(s.scene==='books'){
    const has=new Set(s.objects.map(o=>o.where));
    for(const sh of [...new Set((s.options.cell||[]).map(([v])=>v.split('-')[0]))]){
      cat(`📚 ${sh}번 책꽂이 칸 — 책이 있는 칸은 진하게`);
      for(let k=1;k<=9;k++){const c=`${sh}-${k}`;arg('cell',c,c,'brick rel'+(has.has(c)?'':' empty'));}
    }
  }else{
    cat('📦 물건 — 지금 장면에 있는 것 (색 = 3D 와 동일)');
    for(const o of s.objects)arg('item',o.name,o.id,'brick',o.color);
    cat('📍 자리');
    for(const [v,t] of (s.options.place||[]))arg('place',t,v,'brick rel');
  }
}

/* 파이프라인 드롭: 삽입(blk) · 재배치(pipe) · 쓰레기통 삭제 (원본) */
let dropMark=null;
function pipelineDnD(){
  const pl=$('#pipeline'), tr=$('#blkTrash');
  const hasT=(e,t)=>[...e.dataTransfer.types].includes(t);
  const blocks=()=>[...pl.querySelectorAll('.blk')].filter(b=>!b.classList.contains('cap'));
  const calcAt=y=>{const bs=blocks();let at=bs.length;
    for(let i=0;i<bs.length;i++){const r=bs[i].getBoundingClientRect();if(y<r.top+r.height/2){at=i;break;}}
    return {at,bs};};
  const markAt=(at,bs)=>{
    if(!dropMark){dropMark=document.createElement('div');dropMark.className='dropMark';}
    if(at>=bs.length){const end=pl.querySelector('.blk.cap.end');if(end)pl.insertBefore(dropMark,end);else pl.append(dropMark);}
    else pl.insertBefore(dropMark,bs[at]);
  };
  pl.addEventListener('dragover',e=>{
    if(!(hasT(e,'blk')||hasT(e,'pipe')))return;
    e.preventDefault();e.dataTransfer.dropEffect=hasT(e,'pipe')?'move':'copy';
    const {at,bs}=calcAt(e.clientY);markAt(at,bs);
  });
  pl.addEventListener('dragleave',e=>{if(e.target===pl&&dropMark)dropMark.remove();});
  pl.addEventListener('drop',async e=>{
    e.preventDefault();const {at}=calcAt(e.clientY);if(dropMark)dropMark.remove();
    if(hasT(e,'blk')){await call('seq_insert',at,e.dataTransfer.getData('blk'));sel=at;}
    else if(hasT(e,'pipe')){const i=+e.dataTransfer.getData('pipe');await call('seq_move',i,at>i?at-1:at);sel=-1;}
  });
  tr.addEventListener('dragover',e=>{if(hasT(e,'pipe')){e.preventDefault();tr.classList.add('hot');}});
  tr.addEventListener('dragleave',()=>tr.classList.remove('hot'));
  tr.addEventListener('drop',async e=>{e.preventDefault();tr.classList.remove('hot');
    if(hasT(e,'pipe')){sel=-1;await call('seq_del',+e.dataTransfer.getData('pipe'));}});
}

/* ---------- 타임바: 블록 단위 진행 (원본) ---------- */
function renderTimebar(s){
  $('#previewHint').textContent='3D에서만 움직입니다'+(s.scene==='books'?' · 책과 책꽂이는 물리 엔진':' · 바닥·도킹에 닿는지 계산으로 검사');
  const rg=$('#tbRange'), now=$('#tbNow'), tm=$('#tbTime'), tk=$('#tbTicks');
  const n=s.seq.length, RA=s.run;
  if(+rg.max!==Math.max(0,n-1))rg.max=Math.max(0,n-1);
  if(changed('tbTicks',[n,RA.blk,RA.running,RA.done])){
    tk.innerHTML='';
    for(let i=0;i<n;i++){const b=document.createElement('span');
      if((RA.running&&i<RA.blk)||RA.done)b.className='done';else if(RA.running&&i===RA.blk)b.className='cur';tk.append(b);}
  }
  if(RA.running){
    if(RA.blk>=0)rg.value=RA.blk;
    /* 지금: 몇 번째 블록 · 어느 책 · 어느 단계 · 어느 칸 (원본: "▶ 3/12 잡기 · 하강") */
    const nw=s.tree?.now, endNow=s.tree?.blocks?.find(b=>b.end)?.cur;
    const nwT=nw?` · ${nw.fix?'⟲ 고치기 ':''}${nw.book}${nw.leaf?' · '+nw.leaf:''}${nw.chip?' · '+nw.chip:''}${nw.next?'  → 다음: '+nw.next:''}`:(s.held?` · 든 것: ${s.held}`:'');
    now.textContent=RA.wait?`⏸ 기다림 — ${RA.wait} · 책꽂이·책을 옮겨 길을 열어 주세요`
      :(endNow?'▶ 마무리 — 건드린 책 고치기':`▶ ${RA.blk+1}/${n||1}  ${s.seq[RA.blk]?.text||''}`)+nwT;
    tm.textContent=`${(RA.elapsed||0).toFixed(1)}초`;
  }else{
    now.textContent=RA.err?`■ ${RA.err}`:RA.done?'✔ 끝':s.planErr?`⚠ ${s.planErr}`:(n?'▶ 미리보기를 누르면 처음 블록부터':'블록 없음');
    tm.textContent=`${s.t.toFixed(1)}초`;
  }
}

/* ---------- 시작 ---------- */
(async function init(){
  META=await(await fetch('/api/meta')).json();
  $('#viewer').src=(META&&META.viewer_url)||'/viewer/';
  /* 전광판 클릭 = 전체 로그 오버레이 (원본) */
  $('#ticker').onclick=()=>$('#logOverlay').classList.remove('hidden');
  $('#logClose').onclick=()=>$('#logOverlay').classList.add('hidden');
  $('#logOverlay').onclick=e=>{if(e.target.id==='logOverlay')$('#logOverlay').classList.add('hidden');};
  /* 탭 (원본) */
  $$('#tabs button').forEach(b=>b.onclick=()=>{
    $$('#tabs button').forEach(x=>x.classList.remove('on'));
    $$('.pane').forEach(x=>x.classList.remove('on'));
    b.classList.add('on');$('#tab-'+b.dataset.tab).classList.add('on');
  });
  $('#btnScenePick').onclick=()=>{sel=-1;call('scene',$('#sceneSel').value);};
  $('#btnRobotPick').onclick=()=>call('robot',$('#robotSel').value);
  $('#btnHomeZero').onclick=()=>call('home');
  $('#btnEstop').onclick=()=>call('stop');
  $('#btnPreview').onclick=()=>{$$('#tabs button')[1].click();call('run');};
  $('#btnStopSim').onclick=()=>call('stop');
  $('#btnExample').onclick=()=>{sel=-1;call('example');};
  $('#btnBlkClear').onclick=async()=>{                 // 블록 일괄 삭제 (원본)
    const n=(ST?.seq||[]).length;
    if(!n){setTicker('파이프라인이 이미 비어 있습니다');return;}
    if(!confirm(`블록 ${n}개를 전부 지울까요? (Ctrl+Z 로 되돌릴 수 있습니다)`))return;
    sel=-1;await call('seq_clear');
  };
  /* Ctrl+Z = 실행취소 (원본) */
  addEventListener('keydown',e=>{
    if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'
       &&!/INPUT|TEXTAREA|SELECT/.test(document.activeElement?.tagName||'')){
      e.preventDefault();sel=-1;call('undo');
    }
  });
  $('#chkPlan').onchange=e=>call('show_plan',e.target.checked);
  /* 블록 나무 모두 접기 / 펼치기 (파이프라인이 길 때) */
  $('#btnFoldAll').onclick=()=>{(ST?.seq||[]).forEach((_,i)=>FOLD['b'+i]=false);FOLD.end=false;HASH.seq=null;render(ST);};
  $('#btnOpenAll').onclick=()=>{(ST?.seq||[]).forEach((_,i)=>FOLD['b'+i]=true);FOLD.end=true;
    for(const b of ST?.tree?.blocks||[])for(const n of b.nodes)FOLD['n'+n.id]=true;HASH.seq=null;render(ST);};
  $('#btnBlkDel').onclick=()=>{if(sel>=0){const i=sel;sel=-1;call('seq_del',i);}};
  $('#btnBlkUp').onclick=async()=>{if(sel>0){await call('seq_move',sel,sel-1);sel--;HASH.seq=null;render(ST);}};
  $('#btnBlkDn').onclick=async()=>{if(sel>=0&&sel<ST.seq.length-1){await call('seq_move',sel,sel+1);sel++;HASH.seq=null;render(ST);}};
  pipelineDnD();
  await refresh();
  setInterval(refresh,250);
})();
window.STUDIO={get state(){return ST;},call};   // 화면 검사용
