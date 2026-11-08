/* ================= ARM STUDIO PRO — frontend ================= */
'use strict';
const $=s=>document.querySelector(s), $$=s=>[...document.querySelectorAll(s)];

let META=null;          // /api/meta
let ST=null;            // 마지막 /api/state
let sel=null;           // 파이프라인 선택 블록 라벨
let nmSel=null;         // 이름·색 대상 (3D 좌클릭으로 선택)
let selPose='base';     // 자세 탭 선택
let expanded=[];        // 우클릭으로 펼친 블록 인덱스 (보기 전용)
let ftJoint='J1';
const HASH={};          // 섹션별 변경 감지

/* ---------- 콘솔 + 전광판 ---------- */
let lastLog='대기 중';
function log(msg,err){
  if(!msg)return;
  const c=$('#console'),d=document.createElement('div');
  const t=new Date().toTimeString().slice(0,8);
  d.innerHTML=`<span class="t">${t}</span>`+(err?`<span class="err"></span>`:'');
  (err?d.querySelector('.err'):d).append(msg);
  c.append(d);c.scrollTop=c.scrollHeight;
  while(c.children.length>300)c.firstChild.remove();
  lastLog=String(msg);setTicker(lastLog);
}
function setTicker(t){
  const s=$('#tickerText');
  if(s.textContent===t)return;          // 같은 내용이면 애니메이션 유지
  s.textContent=t;
  s.style.animation='none';void s.offsetWidth;s.style.animation='';
}

/* ---------- API ---------- */
/* 자세 삭제 확인. 'xxx_이동(x,y,z)' 자세는 그 XY 자리의 보정이라(2026-10-20)
   한 번 더 묻고, 아니라고 하면 지우지 않는다. */
function poseDelOk(name){
  // 11-05 (사용자): 보간은 안 쓴다 — '보정 좌표까지 삭제할까요?' 두 번째 질문은 없앴다 (한 번만 묻는다)
  return confirm(`자세 '${name}' 를 삭제할까요?`);
}
async function call(fn,...args){
  try{
    const r=await fetch('/api/call',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({fn,args})});
    const j=await r.json();
    if(!j.ok){log(`${fn}: ${j.error||'실패'}`,true);return null;}
    if(j.job){await pollJob();}
    else if(j.status)log(j.status);
    await refresh();
    return j;
  }catch(e){log(`${fn}: ${e}`,true);return null;}
}
async function pollJob(){
  setBusy(true);let last='';
  for(;;){
    /* 예전엔 250ms 를 쉬고 그 뒤에 요청 두 개(job+state)를 더 했다.
       -> 한 바퀴가 0.3초쯤 되어 시간 표시가 0.3초씩 널뛰었고,
          블록 하이라이트도 그만큼 늦게 따라왔다.
       요청이 2ms/4ms 로 매우 싸므로 훨씬 자주 돌아도 된다. */
    await new Promise(r=>setTimeout(r,80));
    let j;try{j=await(await fetch('/api/job')).json();}catch(e){continue;}
    const s=j.busy?(j.status||j.prog):j.status;
    if(s&&s!==last){log(s);last=s;}
    setTicker(j.busy?(j.prog||j.status||'작업 중'):(s||'완료'));
    // 실행 중에도 상태를 갱신해야 '지금 실행 중인 블록' 하이라이트와
    // 타임바가 따라간다 (예전엔 작업이 끝나야 갱신돼서 안 보였다)
    try{await refresh();}catch(e){}
    if(!j.busy){if(j.error)log(j.error,true);break;}
  }
  setBusy(false);
}
function setBusy(b){
  $('#ticker').classList.toggle('busy',b);
  ['#btnRun','#btnPreview','#btnPreview2','#btnFtGo','#btnPegGo','#btnGrasp','#btnRelease']
    .forEach(s=>{const e=$(s);if(e)e.disabled=b;});
}
async function refresh(){
  const q=new URLSearchParams({sel:sel||'',pose:selPose||''});
  try{ST=await(await fetch('/api/state?'+q)).json();render(ST);}catch(e){}
}
async function fetchMap(){
  const mb=$('#mapBox');
  if(!mb)return;                        // 맵 UI 는 현재 숨김 상태
  try{mb.innerHTML=
    await(await fetch('/api/map?sel='+encodeURIComponent(sel||''))).text();
  }catch(e){}
}
function changed(key,data){
  const j=JSON.stringify(data);
  if(HASH[key]===j)return false;HASH[key]=j;return true;
}

/* ---------- 렌더 ---------- */
function opts(el,list,cur){
  el.innerHTML=list.map(v=>`<option${v===cur?' selected':''}>${esc(v)}</option>`).join('');
}
/* 드래그용 투명 1px 이미지 + 서버에 드래그 중인 설비 id 알림 */
const DRAG_BLANK=new Image();
DRAG_BLANK.src='data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7';
/* [크롬 우회] 크롬은 '다른 출처 iframe'(뷰어 7870, 앱 7900) 으로의 드롭을
   막는다. 파이어폭스는 허용해서 지금까지 잘 됐다 - 코드가 바뀐 게 아니라
   브라우저를 옮기면서 드러난 것이다.
   -> 드래그하는 동안만 투명 덮개를 씌워 '부모 화면' 이 드롭을 받고,
      화면좌표를 뷰어에 postMessage 로 넘긴다 (출처가 달라도 된다). */
let _dropId='';
function vmsg(m){
  const vf=document.querySelector('#viewer');
  if(vf&&vf.contentWindow)vf.contentWindow.postMessage(m,'*');
}
function dropOverlay(on,id){
  const o=document.querySelector('#viewerDrop'); if(!o)return;
  _dropId=on?(id||''):'';
  o.classList.toggle('on',!!on);
  if(!on)vmsg({type:'dragoff'});
}
function _bindDrop(){
  const d=document.querySelector('#viewerDrop'); if(!d)return;
  /* ★ 여기가 핵심 ★
     HTML 드래그앤드롭은 '기본 동작이 거부' 다. dragover 에서 preventDefault()
     를 부르지 않으면 브라우저는 그 자리를 '못 놓는 곳' 으로 보고 금지 표시(X)
     를 띄우며, drop 이벤트를 아예 발생시키지 않는다.
     문서 전체에 preventDefault 를 걸어 어디서든 놓을 수 있게 한다. */
  /* 위치 전달은 '문서 전체' 에서 한다.
     크롬은 커서가 iframe 위에 있을 때 dragover 를 덮개가 아니라 문서로
     보내기도 한다. 덮개에서만 보내면 그 동안 갱신이 끊겨, 뷰어의 800ms
     타임아웃에 걸려 유령이 사라졌다 나타났다 한다(= 뚝뚝 끊김).
     좌표가 실제로 바뀔 때만 보내 헛일도 줄인다. */
  let _lx=-1,_ly=-1;
  const _send=e=>{
    if(!_dropId)return;
    const x=e.clientX,y=e.clientY;
    if(x===_lx&&y===_ly)return;
    _lx=x;_ly=y;
    vmsg({type:'dragmove',x:x,y:y,id:_dropId});
  };
  document.addEventListener('dragover',e=>{
    e.preventDefault();
    try{e.dataTransfer.dropEffect='copy';}catch(_){}
    _send(e);
  });
  document.addEventListener('drop',e=>e.preventDefault());
  d.addEventListener('dragover',e=>{
    e.preventDefault();
    try{e.dataTransfer.dropEffect='copy';}catch(_){}
    _send(e);
  });
  d.addEventListener('drop',e=>{
    e.preventDefault();
    vmsg({type:'dragdrop',x:e.clientX,y:e.clientY,id:_dropId});
    dropOverlay(false);
  });
}
// app.js 가 늦게 실행되면 DOMContentLoaded 는 이미 지나갔을 수 있다
if(document.readyState==='loading')addEventListener('DOMContentLoaded',_bindDrop);
else _bindDrop();
const dragPing=(id,mv)=>fetch('/api/call',{method:'POST',
  headers:{'Content-Type':'application/json'},
  body:JSON.stringify({fn:'env_drag',args:[id,!!mv]})}).catch(()=>{});
/* 복제본 id(도킹1#2) -> 원본 id (썸네일 이미지용) */
const _cloneBase=id=>String(id).split('#')[0];
const DOCK_BASE='Industry_gyu_arm_20260727_-_docking_station-5';   // 도킹 스테이션 원본 STL (견본용)
/* 11-05 (사용자): 3D 미리보기 속도 = 실행 속도 × 미리보기 배속 (실물 실행에는 안 곱한다) */
function pvSpeed(){
  const b=parseFloat($('#speed')?.value||'1.5')||1.5, k=parseFloat($('#pvBoost')?.value||'1')||1;
  return (b*k).toFixed(1)+'x';
}

/* 바닥 플레이트(지형)만 매칭 - seed_plate 같은 일반 설비는 제외 */
const isFloorPlate=id=>/plate_bottom|plate_tile/i.test(id);
/* 배경색에 따라 읽히는 글자색 (밝은 색 위엔 검정) */
function txtColor(hex){
  try{
    const n=parseInt(String(hex).slice(1),16);
    const r=n>>16&255,g=n>>8&255,b=n&255;
    return (0.299*r+0.587*g+0.114*b)>150?'#111':'#fff';
  }catch(e){return '#fff';}
}
function shortNm(id,names){
  return names?.[id]?.name||
    String(id).replace(/Industry_gyu_arm_\d*_?/,'').replace(/^\d{8}_?/,'')
      .replace(/_-_/g,'').replace(/^[-_ ]+/,'')||id;
}
function opts2(el,list,cur,names){   // value=id, 표시=짧은 이름
  el.innerHTML=list.map(v=>`<option value="${esc(v)}"${v===cur?' selected':''}>${
    esc(shortNm(v,names))}</option>`).join('');
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

function render(s){
  /* 닭 머리 표가 새로 만들어졌으면 로컬 조각을 다시 받는다 */
  if(window._headGenCheck)window._headGenCheck(s);
  /* 상단 전광판: 작업 중이면 진행 상황, 아니면 마지막 로그 */
  if(s.busy||s.running)setTicker(s.prog||lastLog);
  const execLab=Object.keys(META.exec_map).find(k=>META.exec_map[k]===s.exec)||s.exec;
  $('#robotChip').textContent=(s.running?'▶ 동작 중 · ':'')+execLab;
  $('#robotChip').classList.toggle('on',s.running||s.busy);
  if(s.estop_held){$('#robotChip').textContent='⛔ 비상정지 · '+$('#robotChip').textContent;}   // 11-06 아두이노 비상정지 버튼
  $('#robotChip').classList.toggle('estop',!!s.estop_held);
  {const pb=$('#btnPause');                      // 11-06 일시 정지 버튼: 실행 중에만, 멈춘 동안엔 [▶ 계속]
   if(pb){pb.disabled=!s.run_active;pb.classList.toggle('on',!!s.paused);
     pb.textContent=s.paused?(s.pause_wait?'▶ 계속 (일시 정지 중 — 3D 에서 도킹을 옮겨도 됩니다)':'▶ 계속 (멈추는 중…)'):'‖ 일시 정지';}}

  /* 자세 목록 */
  if(changed('poses',[s.poses.map(p=>p.name),selPose,s.active_arm,s.arms])){
    const el=$('#poseList');el.innerHTML='';
    /* 11-05 (사용자): base 는 두 팔 공통 자세라 팔마다 하나씩 — 누르면 그 팔로 바뀌고 base 선택 */
    Object.keys(s.arms||{A:''}).forEach(L=>{
      const b=document.createElement('button');
      b.textContent=`${L}팔 base`;
      if(selPose==='base'&&s.active_arm===L)b.classList.add('on');
      b.onclick=async()=>{selPose='base';HASH.sliders=null;
        if(s.active_arm!==L)await call('arm_edit',L);
        await call('select_pose','base');};
      el.append(b);
    });
    s.poses.forEach(p=>{
      if(p.name==='base')return;                 // 위에서 팔마다 따로 보였다
      const b=document.createElement('button');
      /* 11-05 (사용자): 어느 팔 자세인지 — [A]/[B]. base 는 두 팔 공통 자세(보정은 팔마다 따로) */
      b.textContent=p.name==='base'?'base (A·B 공통 · 보정은 팔마다)'
        :(/^[A-Z]팔/.test(p.name)?p.name:`[${p.arm||'A'}팔] ${p.name}`);   // 이름에 팔이 없을 때만 붙인다
      if(p.name===selPose)b.classList.add('on');
      b.onclick=async()=>{selPose=p.name;HASH.sliders=null;
        await call('select_pose',p.name);};
      // 블록에서 등록한 자세가 쌓이므로 여기서 바로 지울 수 있게
      if(p.name!=='base'){
        b.title='우클릭 = 이 자세 삭제';
        b.oncontextmenu=async e=>{e.preventDefault();
          if(!poseDelOk(p.name))return;
          if(selPose===p.name)selPose='base';
          HASH.poses=null;await call('del',p.name);};
        const x=document.createElement('span');
        x.className='poseDel';x.textContent='×';x.title='이 자세 삭제';
        x.onclick=async e=>{e.stopPropagation();
          if(!poseDelOk(p.name))return;
          if(selPose===p.name)selPose='base';
          HASH.poses=null;await call('del',p.name);};
        b.append(x);
      }
      el.append(b);
    });
  }
  /* 블록 팔레트 (스크래치식) */
  renderPalette(s);
  /* 슬라이더 값 */
  const pd=s.poses.find(p=>p.name===selPose);
  if(pd&&changed('sliders',[selPose,pd.deg])){
    $$('#sliders input').forEach((r,i)=>{r.value=pd.deg[i];
      r.closest('.sliderRow').querySelector('.jval').textContent=pd.deg[i]+'°';});
  }
  if(changed('ftoff',s.ft_off))$('#ftOff').textContent=s.ft_off||'';
  /* 11-05 (사용자): 그리퍼 수평 (실물 j4 영점, 지금 팔) */
  if(changed('lvtrim',[s.level_trim,s.active_arm])){
    const v=parseFloat((s.level_trim||{})[s.active_arm||'A']||0);
    const el=$('#lvTrim'); if(el)el.textContent=(v>=0?'+':'')+v.toFixed(1)+'°';
  }
  /* 11-05 (사용자): 미세조정 — 지금 팔 표시 + A/B 고르기 (보정·[로봇을 이 자세로]·[보정 저장] 은 이 팔에) */
  if(changed('ftarm',[s.active_arm,s.arms,selPose])){
    const L=s.active_arm||'A', col=(s.arms||{})[L]||'';
    const fa=$('#ftArm');
    if(fa)fa.textContent=`지금 보정하는 팔: ${L}팔${col?' · '+col+' 팔':''}`
      +(selPose==='base'?' — base 는 두 팔 공통 자세, 보정값은 이 팔 것만 바뀝니다'
                        :' (고른 자세의 팔 — 자세를 고르면 자동으로 바뀝니다)');
    const sel=$('#ftArmSel');
    if(sel&&false){                                  // 11-05: A/B 고르기 줄은 없앰 (자세가 팔을 정한다)
      sel.innerHTML='';
      Object.keys(s.arms||{A:''}).forEach(k=>{
        const bb=document.createElement('button');
        bb.textContent=`${k}팔${(s.arms||{})[k]?' · '+s.arms[k]:''}`;
        if(k===L)bb.classList.add('on');
        bb.onclick=()=>call('arm_edit',k);
        sel.append(bb);
      });
    }
  }

  /* 보정 로봇 / 조인트 */
  if(changed('cal',[s.cal,s.cal_joints,ftJoint])){
    const cur=Object.keys(META.cal_map).find(k=>META.cal_map[k]===s.cal);
    segBtns($('#calRobot'),Object.keys(META.cal_map),cur,
      l=>call('cal_robot',l));
    if(!s.cal_joints.includes(ftJoint))ftJoint=s.cal_joints[0]||'J1';
    segBtns($('#ftJoints'),s.cal_joints,ftJoint,
      l=>{ftJoint=l;HASH.cal=null;render(ST);});
  }

  renderTimebar(s);
  /* 파이프라인 (스크래치식 이중 블록: [로봇][동작][사물 슬롯]) */
  if(changed('seq',[s.seq.map(x=>x.label),sel,s.reach,s.names,s.arms,
                   s.run_at?.blk,s.run_at?.sub,s.seek_blk,
                   expanded.join(',')])){
    const el=$('#pipeline');el.innerHTML='';
    /* 스크래치처럼 시작/끝 블록이 고정 - 그 사이에 꽂는다 */
    const cap=(cls,txt)=>{const c=document.createElement('div');
      c.className='blk cap '+cls;c.textContent=txt;el.append(c);};
    cap('start','🚩 시작');
    if(!s.seq.length){
      const h=document.createElement('div');h.className='hint';
      h.style.padding='.4rem 0';
      h.textContent='팔레트에서 블록을 끌어다 여기 꽂으세요.';
      el.append(h);
    }
    let curArm=Object.keys(s.arms||{A:''})[0]||'A';   // 팔 블록이 나오기 전 = 첫 팔
    {const cur=s.seq.filter(x=>x.kind!=='move').map(blkKey), a=BLK_ANIM.prev;
     if(a&&Date.now()<BLK_ANIM.arm&&a.join('\n')!==cur.join('\n')){
       let p=0;while(p<a.length&&p<cur.length&&a[p]===cur[p])p++;
       let q=0;while(q<a.length-p&&q<cur.length-p&&a[a.length-1-q]===cur[cur.length-1-q])q++;
       BLK_ANIM.idx=new Map();for(let i=p;i<cur.length-q;i++)BLK_ANIM.idx.set(i,i-p);
       BLK_ANIM.start=performance.now();BLK_ANIM.prev=null;
     }}
    const animT=(performance.now()-BLK_ANIM.start)/1000;
    const animOn=BLK_ANIM.idx.size>0&&animT<BLK_ANIM.idx.size*BLK_STEP+0.6;   // 미리보기로 다시 그려도 이어서
    const anim=(d,k,dt=0)=>{if(animOn&&BLK_ANIM.idx.has(k)){d.classList.add('blkIn');
      d.style.animationDelay=(BLK_ANIM.idx.get(k)*BLK_STEP+dt-animT)+'s';}};
    let bOrd=-1;                                   // 화면 블록 순번 (┃이동 줄 제외)
    s.seq.forEach((it,si)=>{
      const d=document.createElement('div');
      {const am=String(it.label||'').match(/^◉\s*\d+\.\s*팔:\s*([A-Z])/);if(am)curArm=am[1];}
      // 라벨의 번호가 진짜 S['seq'] 인덱스다. 화면 목록에는 '┃이동' 연결선이
      // 섞여 있어 forEach 인덱스(si)를 쓰면 한 칸씩 밀린다 (세부 단계가 옆
      // 블록에 붙던 버그).
      const bi=(()=>{const m=String(it.label||'').match(/^[■◆◈➜▼▲◉⧗]\s*(\d+)\./);
                     return m?+m[1]:-1;})();
      const RA=s.run_at||{}, exp=(bi>=0?(s.seq_exp||[])[bi]:null)||{};
      const isRun=RA.running&&RA.blk===bi;
      const isSeek=!RA.running&&s.seek_blk===bi;   // 타임바로 고른 블록
      if(it.kind==='move'){                       // ┃ 연결선은 그대로
        d.className='blk move';d.textContent=it.label;anim(d,bOrd+1,-0.08);
        if(it.label===sel)d.classList.add('on');
        d.onclick=()=>{sel=(sel===it.label)?null:it.label;refresh();};
        el.append(d);return;
      }
      const p=blockParts(it);
      // 잡기·놓기 칸에도 사물 블록(이름)을 그대로 (11-04, 사용자: 색으로만 구분하지 말 것)
      if((p.slotKind==='brick'||p.slotKind==='dock')&&it.target)p.slot=shortNm(it.target,s.names);
      d.className='blk '+p.cls;bOrd++;anim(d,bOrd);
      if(it.label===sel)d.classList.add('on');
      d.draggable=true;                           // 재배치/쓰레기통용
      const acol=(s.arms||{})[curArm]||'';
      d.innerHTML=`<span class="rbadge" title="이 블록을 하는 팔 (팔 블록으로 바꾼다)">${esc(curArm)}${acol?' '+esc(acol):''}</span>
        <span class="act">${esc(p.act)}</span>`;
      if(ARM_CSS[acol]){const rb=d.querySelector('.rbadge');
        rb.style.background=ARM_CSS[acol];rb.style.color=txtColor(ARM_CSS[acol]);}
      const slot=document.createElement('span');
      slot.className='slot '+p.slotKind;slot.textContent=p.slot||'◯';   // 빈칸 (사물을 아직 안 끼움)
      // 슬롯도 그 사물의 '사용자 지정 색'으로 (3D·알약과 같은 색)
      const scol=it.target&&s.names[it.target]?.color;
      if(scol){slot.style.background=scol;slot.style.color=txtColor(scol);}
      d.append(slot);
      /* 도달 불가 = 왼쪽 위 빨간 느낌표 (VS Code 에러 느낌) */
      const rch=s.reach&&s.reach[it.label];
      if(rch&&rch.ok===false){
        const eb=document.createElement('span');
        eb.className='errBadge';eb.textContent='!';
        const why=rch.why||'현재 로봇 위치에서 도달 불가';
        eb.title=why+' (클릭 = 로그에 남기기)';
        eb.onclick=e=>{e.stopPropagation();       // 클릭 = 로그에 기록
          log('⚠ '+it.label+' — '+why,true);};
        eb.onmouseenter=()=>setTicker('⚠ '+it.label+' — '+why);  // 올리면 즉시
        eb.onmouseleave=()=>setTicker(lastLog);
        d.append(eb);
      }
      if(isRun)d.classList.add('running');
      if(isSeek)d.classList.add('seeked');
      /* 우클릭 = 메뉴 (이 자세로 이동 / 자세 등록 / 펼치기) */
      d.oncontextmenu=e=>{e.preventDefault();e.stopPropagation();
        blkMenu(e,bi,it,exp);};
      d.title='우클릭 = 이 자세로 이동 · 자세 등록 · 펼치기';
      if(exp.steps&&exp.steps.length){
        if(true){   // 잡기/놓기 세부 단계는 항상 표시
          const ss=document.createElement('div');ss.className='subSteps';
          exp.steps.forEach(st=>{
            const b=document.createElement('span');
            // 지금 실행 중인 세부 단계만 주황으로 (열기/하강/닫기/상승)
            b.className='subStep'+((isRun&&RA.sub===st)?' on':'');
            b.textContent=st;ss.append(b);
          });
          d.append(ss);
        }
      }
      d.onclick=()=>{sel=(sel===it.label)?null:it.label;refresh();};
      d.ondragstart=e=>{
        e.dataTransfer.setData('pipe',it.label);
        e.dataTransfer.setData('text/plain','');
        e.dataTransfer.effectAllowed='move';
      };
      if(p.slotKind==='down'){                    // 하강: 조작 없음 (원초 블록)
      }else if(p.slotKind==='arm'){               // 팔: 클릭 = 다음 팔로
        slot.title='클릭 = 다른 팔로';slot.style.cursor='pointer';
        slot.onclick=async e=>{e.stopPropagation();HASH.seq=null;
          const ks=Object.keys(s.arms||{A:1,B:1});
          const cur=(p.slot.match(/^([A-Z])/)||[])[1];
          const nx=ks[(ks.indexOf(cur)+1)%ks.length];
          await call('seq_set',it.label,'@arm:'+nx);};
      }else if(p.slotKind==='wait'){              // 기다리기: 클릭 = 초 바꾸기
        slot.title='클릭 = 기다리는 시간 바꾸기';slot.style.cursor='pointer';
        slot.onclick=async e=>{e.stopPropagation();
          const cur=parseFloat(p.slot)||10;
          const v=prompt('몇 초 기다릴까요? (0~600)',String(cur));
          if(v===null)return;const n=parseFloat(v);
          if(!(n>=0&&n<=600)){log('0~600 사이 숫자를 넣으세요',true);return;}
          HASH.seq=null;await call('seq_set',it.label,'@wait:'+n);};
      }else if(p.slotKind==='grip'){              // 그리퍼: 클릭 = 열기/닫기 전환
        slot.title='클릭 = 열기/닫기 전환';slot.style.cursor='pointer';
        slot.onclick=async e=>{e.stopPropagation();HASH.seq=null;
          await call('seq_set',it.label,
            p.slot.includes('열')?'@close':'@open');};
      }
      if(p.slotKind!=='down'&&p.slotKind!=='grip'){   // 11-03: 팔·기다리기 블록도 조각(A팔 · A팔 끝날 때까지)을 받는다
        // 사물 조각을 '블록 어디에든' 떨어뜨리면 슬롯 교체 (덮어쓰기)
        // - 슬롯(작은 알약)만 과녁이면 조준이 어려워서 블록 전체가 받는다
        // - 'obj' 슬롯(사물 이동)은 물체/도킹 어느 쪽이든 받는다
        const okK=p.slotKind==='obj'?['k-brick','k-dock']:['k-'+p.slotKind];
        d.ondragover=e=>{
          const ts=[...e.dataTransfer.types];
          if(!okK.some(t=>ts.includes(t)))return;
          e.preventDefault();e.stopPropagation();
          e.dataTransfer.dropEffect='copy';
          slot.classList.add('hot');
          if(dropMark)dropMark.remove();          // 삽입선 대신 교체 표시
        };
        d.ondragleave=()=>slot.classList.remove('hot');
        d.ondrop=async e=>{
          const a=JSON.parse(e.dataTransfer.getData('arg')||'{}');
          const okKind=p.slotKind==='obj'?(a.k==='brick'||a.k==='dock')
                                         :(a.k===p.slotKind);
          if(!a.v||!okKind)return;                 // 종류 안 맞으면 통과(삽입으로)
          e.preventDefault();e.stopPropagation();
          slot.classList.remove('hot');
          const item=p.slotKind==='obj'?'@goto:'+a.v:
                     p.slotKind==='pose'?a.v:
                     p.slotKind==='arm'?'@arm:'+a.v:
                     p.slotKind==='wait'?'@wait:'+a.v:
                     p.slotKind==='brick'?'@grasp:'+a.v:'@release:'+a.v;
          HASH.seq=null;await call('seq_set',it.label,item);
          const mi=it.label.match(/(\d+)\./);         // 교체한 블록까지 미리보기
          if(mi)previewUpto(+mi[1]);
        };
      }
      el.append(d);
    });
    cap('end','🏁 끝');
  }
  /* 선택 블록 패널 (2026-08-13 화면에서 숨김 - 요소가 있을 때만 동작) */
  const kind=s.seq.find(x=>x.label===sel)?.kind;
  $('#segPanel')?.classList.toggle('hidden',kind!=='move');
  $('#ignPanel')?.classList.toggle('hidden',kind!=='pose');
  if($('#segMode')&&kind==='move'&&s.seg&&changed('seg',s.seg)){
    $('#segInfo').textContent=`${s.seg.a} → ${s.seg.b}`;
    opts($('#segMode'),['(기본값 사용)',META.mode_simple,...META.modes],
      s.seg.mode||'(기본값 사용)');
    opts($('#segAx'),['(기본값 사용)',...META.axorders],s.seg.ax||'(기본값 사용)');
    opts($('#segRot'),META.rots,s.seg.rot);
    segBtns($('#segPrefer'),['x 먼저','y 먼저'],
      s.seg.prefer?(s.seg.prefer==='x'?'x 먼저':'y 먼저'):null,
      l=>call('seg_prefer',sel,l));
    $('#segRec').textContent=
      (s.seg.rec?`녹화 ${s.seg.rec}프레임 · `:'')+
      (s.seg.path_pts?`그린 경로 ${s.seg.path_pts}점`:'경로 없음');
  }
  if($('#ignList')&&kind==='pose'&&changed('ign',[s.obstacles,s.ign_sel])){
    const el=$('#ignList');el.innerHTML='';
    s.obstacles.forEach(id=>{
      const l=document.createElement('label');l.className='chk';
      l.innerHTML=`<input type="checkbox" value="${esc(id)}"${
        (s.ign_sel||[]).includes(id)?' checked':''}>${esc(id)}`;
      el.append(l);
    });
  }

  /* 실행 로봇 */
  if(changed('exec',s.exec)){
    const cur=Object.keys(META.exec_map).find(k=>META.exec_map[k]===s.exec);
    segBtns($('#execSel'),META.exec_labels,cur,l=>call('exec_robot',l));
  }
  /* 매크로 */
  if(changed('macros',s.macros)){
    const el=$('#macroBtns');el.innerHTML='';
    s.macros.forEach((lab,i)=>{
      const b=document.createElement('button');b.className='btn wide';
      b.textContent=lab;b.onclick=()=>call('macro_go',META.macro_slots[i]);
      el.append(b);
    });
  }
  /* 간편 이동 설정 (숨김 - 요소가 있을 때만) */
  if($('#spPrefer')&&changed('simple',s.simple)){
    segBtns($('#spPrefer'),['x 먼저','y 먼저'],
      s.simple.prefer==='x'?'x 먼저':'y 먼저',l=>simpleSet(l));
    $('#spClear').value=s.simple.clear_cm;$('#spClearV').value=s.simple.clear_cm;
    $('#spExtra').value=s.simple.extra_cm;$('#spExtraV').value=s.simple.extra_cm;
  }

  /* USB 연결 자동 감지 - 표시/로그만 한다 (동작에는 관여 안 함) */
  if(changed('ports',s.ports)){
    const st=s.ports?.sts, dx=s.ports?.dxl;
    const txt=(!st&&!dx)?'로봇 없음 — 시뮬레이션만 가능'
      :[st?'SO-ARM':null,dx?'Dynamixel':null].filter(Boolean).join(' + ')+' 연결됨';
    const el=$('#connChip');
    el.textContent='🔌 '+txt;
    el.classList.toggle('on',!!(st||dx));
    log('[연결 감지] '+txt+(st?` (SO-ARM ${st})`:'')+(dx?` (DXL ${dx})`:''));
  }

  /* 설비/물체 카드: '견본(끌어서 새로 배치)' 과 '배치됨(현재 공장)' 구분
     - 견본 드래그 = 새 인스턴스 생성 (도킹#2, ppap#2 ...)
     - 배치됨 드래그 = 그 인스턴스를 그대로 이동 (복제 아님)
     - 카드 색 = 실제 3D 색과 동기화 (지정 안 했으면 3D 기본색) */
  if(changed('env',[s.env,s.env_on,s.names,s.bricks,
                    s.bricks_placed,s.env_colors])){
    const el=$('#envList');el.innerHTML='';
    const colOf=id=>s.names[id]?.color||s.env_colors?.[id]||'#e0c020';
    const card=(id,badge,mv)=>{
      const nm=shortNm(id,s.names);
      const c=document.createElement('div');c.className='thumbCard';
      c.draggable=true;
      c.title=mv?(id+' — 끌어서 위치 이동 · 3D 우클릭=삭제')
                :(id+' — 끌어놓을 때마다 새 인스턴스 생성');
      // 견본(템플릿)에는 '색' 개념이 없다 - 색은 배치된 실물의 속성
      const col=mv?colOf(id):'#9aa4b2';
      c.innerHTML=`<img src="/api/thumb/${encodeURIComponent(_cloneBase(id))}?c=${
          encodeURIComponent(col)}" draggable="false">
        <div class="thumbNm">${badge}${esc(nm)}</div>`+
        (mv?`<input type="color" class="thumbCol" value="${col}" title="색 지정">`:'');
      const cp=c.querySelector('.thumbCol');
      if(cp){
        cp.onchange=ev=>{
          HASH.env=null;
          call('set_name',id,s.names[id]?.name||'',ev.target.value);
        };
        cp.onpointerdown=ev=>ev.stopPropagation();
      }
      c.ondragstart=e=>{
        e.dataTransfer.setData('text/plain',id);
        const hex=[...new TextEncoder().encode(id)]
          .map(b=>b.toString(16).padStart(2,'0')).join('');
        e.dataTransfer.setData('envid-'+hex,id);
        // ★ 이게 없으면 브라우저가 드롭을 거부한다 ★
        //   effectAllowed 와 dragover 쪽 dropEffect 가 맞아야 drop 이 뜬다.
        //   여기만 빠져 있어서 설비/물체를 놓아도 drop 이 안 왔고,
        //   서버는 '드래그 취소' 로 보고 복제본을 지웠다(= 갑자기 사라짐).
        e.dataTransfer.effectAllowed='all';
        e.dataTransfer.setDragImage(DRAG_BLANK,0,0);
        dropOverlay(true,id);            // 크롬용 덮개 켜기
        dragPing(id,mv);                 // mv=true 면 복제 없이 이동
      };
      c.ondragend=()=>{dropOverlay(false);dragPing('');};
      c.dataset.id=id;
      el.append(c);
    };
    const capt=t=>{const h=document.createElement('div');
      h.className='palCat';h.textContent=t;el.append(h);};
    /* ① 견본 (원본만) */
    capt('🧰 견본 — 끌어서 공장에 배치 (놓을 때마다 새로 하나)');
    /* 11-05 (사용자): 도킹 스테이션도 늘 견본으로 — 3개를 쓸지 4개를 쓸지는 사용자가 끌어다 정한다.
       복제본(#2, #8 …)만 있고 원본이 목록에 없으면 원본 이름으로 견본을 만든다 (서버 env_clone 이 복제) */
    const tmpl=[];
    s.env.filter(id=>!isFloorPlate(id)).forEach(id=>{
      const b=_cloneBase(id); if(!tmpl.includes(b))tmpl.push(b);});
    if(!tmpl.some(b=>/docking/i.test(b)))tmpl.push(DOCK_BASE);
    tmpl.sort((a,b)=>(/docking/i.test(b)?1:0)-(/docking/i.test(a)?1:0));   // 도킹 먼저
    tmpl.forEach(id=>card(id,'',false));
    s.bricks.filter(b=>!b.includes('#')).forEach(id=>card(id,'📦 ',false));
    /* ② 배치됨 (공장에 실제로 있는 인스턴스: 보이는 설비 + 놓인 물체) */
    const placedEnv=s.env_on.filter(id=>!isFloorPlate(id));
    const placedAll=[...placedEnv,...(s.bricks_placed||[])];
    if(placedAll.length){
      capt('📍 배치됨 — 끌면 이동 · 3D 우클릭=삭제');
      placedEnv.forEach(id=>card(id,'',true));
      (s.bricks_placed||[]).forEach(id=>card(id,'📦 ',true));
    }
  }
  if(changed('plates',[s.plates,s.plate_info])){
    opts($('#plateSel'),s.plates,$('#plateSel').value);
    $('#plateInfo').textContent=s.plate_info||'(없음)';
  }
  if(changed('rpos',s.robot_pos))$('#robotPos').textContent=s.robot_pos;
  /* 공장 배치 프리셋 목록 */
  if(changed('fps',[s.factory_presets,s.last_preset])){
    opts($('#fpSel'),s.factory_presets||[],s.last_preset||$('#fpSel').value);
    /* 11-05 (사용자): 지금 배치 표시 — 이름을 비우고 [저장] = 여기에 덮어쓰기 */
    const fc=$('#fpCur');
    if(fc)fc.textContent=s.last_preset?`지금 배치: ${s.last_preset}  ·  이름 비우고 [저장] = 덮어쓰기`:'';
  }
  // 로봇 배치가 확정되면 서버가 이동 모드를 끄므로 체크박스도 따라간다
  if(changed('rmoveOn',s.rmove_on))$('#robotMove').checked=!!s.rmove_on;

  /* 공간 선택 (숨김 - 요소가 있을 때만) */
  if($('#pegView')&&changed('peg',s.peg)){
    $('#pegView').checked=s.peg.on;
    opts($('#pegSel'),s.peg.choices,s.peg.sel||s.peg.choices[0]);
  }
  if(changed('bricks',[s.bricks,s.docking,s.names])){
    if($('#brickSel'))opts2($('#brickSel'),s.bricks,$('#brickSel').value,s.names);
    if($('#dockSel'))opts2($('#dockSel'),s.docking,$('#dockSel').value,s.names);
    if(nmSel)$('#nmCur').textContent=shortNm(nmSel,s.names);   // 이름 갱신 반영
  }
}

/* ---------- 간편 이동 설정 전송 (UI 숨김 중 - 요소 있을 때만) ---------- */
function simpleSet(preferLab){
  if(!$('#spPrefer'))return;
  const p=preferLab||($('#spPrefer .on')?.textContent)||'x 먼저';
  call('simple_set',p,+$('#spClear').value,+$('#spExtra').value).then(fetchMap);
}

/* ---------- 블록 팔레트: 동작 블록 + 사물 조각 (스크래치식) ---------- */
function blockParts(it){
  /* 라벨('➜ 3. 이동: 도킹1' 등) -> {동작, 슬롯 내용, 슬롯 종류, 색 클래스} */
  const m=it.label.match(/^([■◆◈➜▼▲◉⧗])\s*\d+\.\s*(.*)$/);
  const body=m?m[2]:it.label;
  if(m&&m[1]==='➜')                       // 사물 중심 이동 (즉석 IK)
    return {act:'이동 →',slot:body.replace(/^이동:\s*/,''),
            slotKind:'obj',cls:'pose'};
  if(m&&m[1]==='■')                       // (레거시) 등록 자세 이동
    return {act:'이동 →',slot:body,slotKind:'pose',cls:'pose'};
  if(m&&m[1]==='◆')
    return {act:'그리퍼',slot:body.includes('열')?'열기 ✋':'닫기 ✊',
            slotKind:'grip',cls:'grip'};
  if(m&&m[1]==='▼')                       // 하강: 그 자리 수직 (원초 블록)
    return {act:'하강 ▼',slot:'잡는 높이까지',slotKind:'down',cls:'grip'};
  if(m&&m[1]==='▲')                       // 상승: 그 자리 수직 (원초 블록)
    return {act:'상승 ▲',slot:'접근 높이까지',slotKind:'down',cls:'grip'};
  if(m&&m[1]==='◉')                       // 팔 바꾸기 (10-30): 이 뒤는 이 팔이 한다
    return {act:'팔 🤖',slot:body.replace(/^팔:\s*/,''),slotKind:'arm',cls:'grip arm'};
  if(m&&m[1]==='⧗')                       // 기다리기 (10-30)
    return {act:'기다리기 ⏱',slot:body.replace(/^기다리기\s*/,''),slotKind:'wait',cls:'grip'};
  if(body.startsWith('잡기'))
    return {act:'잡기',slot:body.replace(/^잡기:\s*/,''),
            slotKind:'brick',cls:'brick'};
  return {act:'놓기',slot:body.replace(/^놓기:\s*/,''),
          slotKind:'dock',cls:'brick rel'};
}
/* ---------- 블록 우클릭 메뉴 ----------
   목적: 미세 자세 조정. [이 자세로 이동] 으로 그 자리까지 가서 멈추고,
   자세가 맞으면 [자세 등록] -> ③연동 탭에서 +/- 미세조정. */
function blkMenu(e,si,it,exp){
  document.querySelectorAll('.blkMenu').forEach(x=>x.remove());
  const mn=document.createElement('div');
  mn.className='blkMenu';
  const lab=(it.label||'').replace(/^[■◆◈➜▼▲◉⧗]\s*\d+\.\s*/,'');
  const add=(txt,fn,hint)=>{
    const b=document.createElement('button');
    b.innerHTML=txt+(hint?`<small>${esc(hint)}</small>`:'');
    b.onclick=async ev=>{ev.stopPropagation();mn.remove();await fn();};
    mn.append(b);
  };
  const head=document.createElement('div');
  head.className='mhead';head.textContent=lab;mn.append(head);
  /* ★ 잡기·놓기는 좌표를 갖지 않는다 ★
     좌표를 가진 블록은 [이동] 하나뿐이다. 잡기/놓기를 자세로 등록하면
     그 자세가 좌표를 갖게 되어 이동과 두 개가 된다(어제 없앤 문제가 재발).
     -> 잡기/놓기 우클릭에서는 '이 자세로 이동' 과 '자세 등록' 을 빼서
        실수로라도 좌표가 둘이 될 수 없게 한다. 미세조정은 [이동] 에서만. */
  //  kind==='brick' 이 잡기·놓기다 (서버가 붙여 주는 값 — 라벨보다 확실).
  //  라벨 기호(◈)도 함께 본다.
  const isGraspRel = it.kind==='brick'
                 || /◈/.test(it.label||'')
                 || /^(잡기|놓기)/.test(lab);
  if(!isGraspRel){
    add('⏩ 이 자세로 (바로 이동)',async()=>{
      await call('seq_seek',si);          // 재생 없이 그 자세로 점프
    },'그 블록 자세로 즉시 — 3D만 움직임');
  }
  add('▶ 여기까지 재생',async()=>{
    setTicker(`${si+1}번 블록까지 재생합니다`);
    await call('seq_goto',si,pvSpeed());
  },'처음부터 이 블록까지 보여주고 그 자리에 멈춤');
  if(!isGraspRel){
    add('🎯 자세 등록 (미세조정용)',async()=>{
      const j=await call('blk_to_pose',it.label);
      if(j&&j.status){
        const m=j.status.match(/'([^']+)'/);
        if(m){selPose=m[1];HASH.poses=null;HASH.sliders=null;await refresh();
          setTicker('자세 등록 완료 → ③연동 탭에서 +/- 미세조정');}
        else setTicker(String(j.status).split('\n')[0]);     // 11-05: 실패 이유를 보인다 (예전엔 말없이 끝남)
        log(String(j.status));
      }
    },'지금 이 자리를 자세로 저장 → ③연동 탭에서 보정');
  }else{
    const note=document.createElement('div');
    note.className='mhead';
    note.style.cssText='font-size:.72rem;color:#8b93a7;font-weight:400';
    note.textContent='잡기·놓기는 칸의 사물 위로 먼저 간 뒤 동작합니다 — 미세조정은 [이동] 블록에서';
    mn.append(note);
  }

  add('🗑 이 블록 삭제',async()=>{
    sel=null;HASH.seq=null;await call('seq_del',it.label);
  });
  document.body.append(mn);
  const r=mn.getBoundingClientRect();
  mn.style.left=Math.min(e.clientX,innerWidth-r.width-8)+'px';
  mn.style.top=Math.min(e.clientY,innerHeight-r.height-8)+'px';
  const close=ev=>{if(!mn.contains(ev.target)){mn.remove();
    document.removeEventListener('mousedown',close);}};
  setTimeout(()=>document.addEventListener('mousedown',close),0);
}

/* ---------- 타임바: 블록 단위 재생 슬라이더 ---------- */
let tbDragging=false;
function renderTimebar(s){
  const rg=$('#tbRange'), now=$('#tbNow'), tm=$('#tbTime'), tk=$('#tbTicks');
  if(!rg)return;
  const n=s.seq.filter(x=>x.kind!=='move').length;
  const RA=s.run_at||{};
  if(+rg.max!==Math.max(0,n-1)){rg.max=Math.max(0,n-1);}
  rg.disabled=!n;
  // 눈금 (블록 개수만큼) — 지난 블록 파랑, 지금 블록 주황
  if(changed('tbTicks',[n,RA.blk,RA.running,s.seek_blk])){
    tk.innerHTML='';
    for(let i=0;i<n;i++){
      const b=document.createElement('span');
      if(RA.running&&i<RA.blk)b.className='done';
      else if(RA.running&&i===RA.blk)b.className='cur';
      else if(!RA.running&&s.seek_blk===i)b.className='cur';
      tk.append(b);
    }
  }
  if(RA.running){
    if(!tbDragging&&RA.blk>=0)rg.value=RA.blk;
    const lab=(s.seq[RA.blk]?.label||'').replace(/^[■◆◈➜▼▲]\s*/,'');
    now.textContent=`▶ ${RA.blk+1}/${n||1}  ${lab}`+(RA.sub?` · ${RA.sub}`:'');
    tm.textContent=`${(RA.elapsed||0).toFixed(1)}초`;
  }else if(!tbDragging){
    const sb=s.seek_blk;
    if(sb>=0&&sb<n){
      rg.value=sb;
      const lab=(s.seq[sb]?.label||'').replace(/^[■◆◈➜▼▲]\s*/,'');
      now.textContent=`⏸ ${sb+1}/${n}  ${lab}  (이 자세)`;
      tm.textContent='정지';
    }else{
      now.textContent=n?'끌면 그 블록 자세로 바로 이동':'블록 없음';
      tm.textContent=`${(RA.elapsed||0).toFixed(1)}초`;
    }
  }
}

/* 팔 색 이름 -> 화면 색 (빨간 팔·파란 팔 …) */
const ARM_CSS={'빨강':'#e53935','주황':'#fb8c00','노랑':'#fdd835','초록':'#43a047','파랑':'#1e88e5','보라':'#8e24aa'};
function renderPalette(s){
  const el=$('#blkPalette');
  if(!el||!changed('palette',[s.poses.map(p=>p.name),s.bricks,s.docking,
                              s.names,s.env_on,s.bricks_placed,s.arms,s.active_arm]))return;
  el.innerHTML='';
  const cat=t=>{const h=document.createElement('div');
    h.className='palCat';h.textContent=t;el.append(h);};
  const act=(cls,label,item)=>{           // 동작 블록 (기본 사물이 끼워진 채 삽입)
    const c=document.createElement('div');
    c.className='blkChip '+cls;c.textContent=label;
    c.draggable=true;c.title='끌어서 파이프라인에 놓기 (사물은 나중에 끼워도 됨)';
    c.ondragstart=e=>{
      e.dataTransfer.setData('blk',item);
      e.dataTransfer.setData('text/plain','');  // 3D 뷰에 잘못 놓여도 무시되게
      e.dataTransfer.effectAllowed='copy';
    };
    el.append(c);
  };
  const arg=(kind,label,val,cls)=>{       // 사물 조각 (슬롯에 끼우는 알약)
    const c=document.createElement('div');
    c.className='argChip '+cls;c.textContent=label;
    // 색 = 사용자가 공장 꾸미기에서 그 사물에 지정한 색 (3D 와 동일)
    const col=s.names[val]?.color;
    if(col){c.style.background=col;c.style.color=txtColor(col);}
    c.draggable=true;
    c.title='동작 블록의 구멍(슬롯)에 끼우거나, 파이프라인에 바로 놓기';
    c.ondragstart=e=>{
      e.dataTransfer.setData('arg',JSON.stringify({k:kind,v:val}));
      e.dataTransfer.setData('k-'+kind,'1');    // dragover 에서 종류 판별용
      e.dataTransfer.setData('text/plain','');
      e.dataTransfer.effectAllowed='copy';
    };
    el.append(c);
  };
  /* 사물 = '공장에 배치된 것만' (숨은 견본은 잡기/이동 대상이 아님) */
  const pDocks=s.docking.filter(d=>s.env_on.includes(d));
  const pBricks=(s.bricks_placed||[]);
  cat('동작 블록 — 파이프라인에 놓기');
  // 11-03: 동작 블록은 늘 보인다 (블록 코딩). 공장에 사물이 없으면 빈칸(◯)으로 들어가고 나중에 사물 조각을 끼운다
  act('pose','이동 → ◯','@goto:'+(pDocks[0]||pBricks[0]||''));   // 이동 = 사물 위로 (즉석 IK)
  act('brick','잡기 ◯','@grasp:'+(pBricks[0]||''));
  act('brick rel','놓기 ◯','@release:'+(pDocks[0]||''));
  act('brick','옮기기 ◯ → ◯',                                        // 잡기+놓기 한 줄 (잡기=이동 포함)
      'PAIR:@grasp:'+(pBricks[0]||'')+'|@release:'+(pDocks[0]||''));
  act('grip','그리퍼 열기 ✋','@open');
  act('grip','하강 ▼','@down');
  act('grip','상승 ▲','@up');
  act('grip','그리퍼 닫기 ✊','@close');
  // 11-03: 팔·기다리기도 사물처럼 '블록 + 조각'. 조각을 끌어 블록 칸(◯)에 끼운다.
  //   팔 위치 옮기기는 3D 에서 팔을 우클릭 (예전 '3D 에서 옮길 팔' 줄은 없앰)
  cat('🤖 팔 · 기다리기 — 팔 블록 뒤의 블록은 그 팔이 한다 · 팔 옮기기 = 3D 에서 팔 우클릭');
  const ARMS=s.arms||{A:'빨강',B:'파랑'};
  act('grip','팔 바꾸기 ◯','@arm:'+(Object.keys(ARMS)[0]||'A'));
  act('grip','기다리기 ◯','@wait:10');
  Object.entries(ARMS).forEach(([L,c])=>{                // 팔 조각 = 그 팔 색 (3D 의 그리퍼 턱 색과 같다)
    arg('arm',`${L}팔`,L,'');
    const last=el.lastChild;last.title=`${L}팔 · ${c} 팔 — 팔 바꾸기 블록 칸에 끼우기`;
    if(ARM_CSS[c]){last.style.background=ARM_CSS[c];last.style.color=txtColor(ARM_CSS[c]);}
  });
  arg('wait','⏱ 10초','10','');el.lastChild.style.background='var(--sc-grip)';
  el.lastChild.title='기다리기 블록 칸에 끼우기 (블록을 누르면 초를 바꾼다)';
  Object.entries(ARMS).forEach(([L,c])=>{                // 'A팔 끝날 때까지'
    arg('wait',`${L}팔 끝날 때까지`,'arm:'+L,'');
    const last=el.lastChild;last.style.background='var(--sc-grip)';
    last.title=`${L}팔이 하던 블록이 다 끝나면 다음으로 — 기다리기 블록 칸에 끼우기`;
    if(ARM_CSS[c])last.style.boxShadow=`inset .3rem 0 0 ${ARM_CSS[c]},0 1.5px 0 rgba(0,0,0,.3)`;
  });
  {const b=document.createElement('button');b.className='btn sm';b.textContent='⚙ 팔 이름·색';
   b.title='예: A=빨강 B=파랑 C=노랑 (말로 "빨간 팔" 처럼 부른다)';
   b.onclick=async()=>{const cur=Object.entries(s.arms||{}).map(([k,v])=>k+'='+v).join(' ');
     const v=prompt('팔 이름=색 (색: 빨강·주황·노랑·초록·파랑·보라)',cur||'A=빨강 B=파랑');
     if(v!==null){HASH.seq=null;HASH.palette=null;await call('arms_set',v);}};
   el.append(b);}
  cat('📦 사물 — 공장에 배치된 것만 (색 = 3D 와 동일)');
  if(!pBricks.length&&!pDocks.length){
    const h=document.createElement('div');h.className='hint';
    h.textContent='배치된 사물이 없습니다 — ①공장 꾸미기에서 끌어다 놓으세요';
    el.append(h);
  }
  pBricks.forEach(b=>arg('brick',shortNm(b,s.names),b,'brick'));
  pDocks.forEach(d=>arg('dock',shortNm(d,s.names),d,'brick rel'));
  (s.bricks||[]).filter(b=>!pBricks.includes(b)&&!b.includes('#')).forEach(b=>{   // 11-03: 등록은 됐지만 공장에 없는 물체 (예: ppap · 복제본 빼고)
    arg('brick',shortNm(b,s.names),b,'brick');
    const c=el.lastChild;c.draggable=false;c.style.opacity='.38';c.style.cursor='not-allowed';
    c.title='공장에 없습니다 — ①공장 꾸미기에서 배치하면 쓸 수 있습니다';
  });
}

/* 블록을 놓은 직후: '그 블록까지' 자동 미리보기 (바쁘면 건너뜀) */
function previewUpto(i){
  if(ST&&(ST.busy||ST.running))return;
  // 11-03: 팔·기다리기 블록까지만이면 미리보기 안 함 — 3D 에 보일 동작이 없는데,
  //   미리보기가 끝나며 기본 자세·A팔로 돌아가서 '팔이 움직이다 원위치' 처럼 보였다
  const b=(ST?.seq||[]).filter(x=>x.kind!=='move').find(x=>new RegExp('^[■◆◈➜▼▲◉⧗]\\s*'+i+'\\.').test(String(x.label||'')));
  if(b&&/^[◉⧗]/.test(b.label))return;
  call('preview_upto',i);
}

/* 말로 만들기로 블록이 생기면 한 번에 뜨지 않고 하나씩 '착' 붙는 모션 (11-03, 사용자) */
const BLK_ANIM={prev:null,arm:0,start:0,idx:new Map()};   // idx: 화면 블록 순번 -> 나오는 차례
const blkKey=x=>String(x.label||'').replace(/^[■◆◈➜▼▲◉⧗]\s*\d+\.\s*/,'');
const BLK_STEP=0.22;                                       // 블록 사이 간격 (초)

/* 파이프라인 드롭: 삽입(blk/arg) · 재배치(pipe) · 쓰레기통 삭제 */
let dropMark=null;
function pipelineDnD(){
  const pl=$('#pipeline'), tr=$('#blkTrash');
  const hasT=(e,t)=>[...e.dataTransfer.types].includes(t);
  const blocks=()=>[...pl.querySelectorAll('.blk')]
    .filter(b=>!b.classList.contains('move')&&!b.classList.contains('cap'));
  const calcAt=y=>{
    const bs=blocks();let at=bs.length;
    for(let i=0;i<bs.length;i++){
      const r=bs[i].getBoundingClientRect();
      if(y<r.top+r.height/2){at=i;break;}
    }
    return {at,bs};
  };
  const markAt=(at,bs)=>{                 // 삽입선은 항상 시작/끝 사이에
    if(!dropMark){dropMark=document.createElement('div');
      dropMark.className='dropMark';}
    if(at>=bs.length){
      const end=pl.querySelector('.blk.cap.end');
      if(end)pl.insertBefore(dropMark,end);else pl.append(dropMark);
    }else pl.insertBefore(dropMark,bs[at]);
  };
  pl.addEventListener('dragover',e=>{
    if(!(hasT(e,'blk')||hasT(e,'arg')||hasT(e,'pipe')))return;
    /* 슬롯 위에 사물을 겹친 중이면 삽입선 대신 슬롯 강조만 */
    if(hasT(e,'arg')&&e.target.classList?.contains('slot')){
      if(dropMark)dropMark.remove();return;
    }
    e.preventDefault();
    e.dataTransfer.dropEffect=hasT(e,'pipe')?'move':'copy';
    const {at,bs}=calcAt(e.clientY);
    markAt(at,bs);
  });
  pl.addEventListener('dragleave',e=>{
    if(e.target===pl&&dropMark)dropMark.remove();
  });
  pl.addEventListener('drop',async e=>{
    e.preventDefault();
    if(dropMark)dropMark.remove();
    const {at}=calcAt(e.clientY);
    HASH.seq=null;
    const pipe=e.dataTransfer.getData('pipe');       // 파이프라인 안 재배치
    if(pipe){sel=null;await call('seq_move_to',pipe,at);return;}
    const blk=e.dataTransfer.getData('blk');         // 동작 블록 삽입
    if(blk){
      if(blk.startsWith('PAIR:')){                   // 옮기기 = 이동+잡기+이동+놓기
        const [a1,a2]=blk.slice(5).split('|');       // 잡기/놓기는 이동을 포함하지
        const g1='@goto:'+a1.split(':')[1];          // 않으므로 이동 블록을 앞에
        const g2='@goto:'+a2.split(':')[1];          // 자동으로 끼워준다
        await call('seq_insert',g1,at);
        await call('seq_insert',a1,at+1);
        await call('seq_insert',g2,at+2);
        await call('seq_insert',a2,at+3);
        previewUpto(at+3);
      }else{
        await call('seq_insert',blk,at);
        if(!/^@(arm|wait):/.test(blk))previewUpto(at);   // 팔·기다리기는 미리보기 안 함 (11-03)
      }
      return;
    }
    const a=JSON.parse(e.dataTransfer.getData('arg')||'{}');
    if(a.v){                                         // 사물을 바로 놓으면 그 동작으로
      const item=a.k==='pose'?a.v:a.k==='arm'?'@arm:'+a.v:a.k==='wait'?'@wait:'+a.v:
                 a.k==='brick'?'@grasp:'+a.v:'@release:'+a.v;
      await call('seq_insert',item,at);
      if(!/^@(arm|wait):/.test(item))previewUpto(at);
    }
  });
  /* 쓰레기통: 파이프라인 블록을 끌어다 놓으면 삭제 */
  tr.addEventListener('dragover',e=>{
    if(!hasT(e,'pipe'))return;
    e.preventDefault();e.dataTransfer.dropEffect='move';
    tr.classList.add('hot');
  });
  tr.addEventListener('dragleave',()=>tr.classList.remove('hot'));
  tr.addEventListener('drop',async e=>{
    e.preventDefault();tr.classList.remove('hot');
    const lab=e.dataTransfer.getData('pipe');
    if(!lab)return;
    HASH.seq=null;sel=null;
    await call('seq_del',lab);
  });
}

/* ---------- 초기화 ---------- */
async function boot(){
  META=await(await fetch('/api/meta')).json();

  /* 뷰어 */
  const host=location.hostname;
  /* ★ 뷰어를 이 페이지(7900)를 거쳐 연다.
       윈도우가 7781~7880 을 예약해 둬서 7870 직결은 브라우저에서
       "연결할 수 없음" 이 난다 (WSL 안에서는 멀쩡한데도).
       같은 출처가 되니 크롬의 드롭 차단도 같이 풀린다. */
  $('#viewer').src=(META&&META.viewer_url)||'/viewer/';

  /* 전광판 클릭 = 전체 로그 오버레이 */
  $('#ticker').onclick=()=>$('#logOverlay').classList.remove('hidden');
  $('#logClose').onclick=()=>$('#logOverlay').classList.add('hidden');
  $('#logOverlay').onclick=e=>{
    if(e.target.id==='logOverlay')$('#logOverlay').classList.add('hidden');};

  /* 탭 */
  let lastTab='factory';
  $$('#tabs button').forEach(b=>b.onclick=()=>{
    if(b.dataset.tab==='spot'){openSpot();return;}   // 11-06: ④ Spot 제어 = 본문을 Spot 화면으로
    $$('#tabs button').forEach(x=>x.classList.remove('on'));
    $$('.pane').forEach(x=>x.classList.remove('on'));
    b.classList.add('on');$('#tab-'+b.dataset.tab).classList.add('on');
    lastTab=b.dataset.tab;
    if(b.dataset.tab==='sim')fetchMap();   // 맵은 ② 시뮬레이션 탭에 있음
  });
  /* ④ Spot 제어 (11-06) — 뮤의 책꽂이 공정 서버(:7910)를 본문 자리에 꽉 차게. 돌아가기는 위쪽 버튼 */
  const SPOT_URL=`${location.protocol}//${location.hostname}:7910/`;
  async function spotAlive(){
    try{await fetch(SPOT_URL+'api/meta',{mode:'no-cors',cache:'no-store'});return true;}catch(e){return false;}
  }
  async function openSpot(){
    document.querySelector('main').classList.add('spot');
    const fr=$('#spotFrame');
    $('#spotMsg').textContent='';
    if(!(await spotAlive())){
      $('#spotMsg').textContent='Spot 서버(:7910)가 꺼져 있습니다 — 뮤에서 bash ~/dusk_tpu/start_spot.sh';
      return;
    }
    if(!fr.getAttribute('src'))fr.setAttribute('src',SPOT_URL);
  }
  $('#spotBack').onclick=()=>{
    document.querySelector('main').classList.remove('spot');
    $('#tabs button[data-tab="'+lastTab+'"]')?.click();
  };
  if(location.hash==='#spot')setTimeout(openSpot,300);   // 주소 끝 #spot = 바로 ④ Spot 제어
  // 11-06 (사용자: spot 에서 원위치 누르면 스팟 공간이 원위치) — Spot 화면일 때 위쪽 [⌂ 원위치] 는 Spot 공간(책꽂이·책·로봇)을
  //   처음 상태로 (Spot 화면 안의 ⌂ 원위치와 같은 'home'). 실물 SO-ARM 은 움직이지 않는다.
  window.spotHome=async()=>{
    try{await fetch(SPOT_URL+'api/call',{method:'POST',mode:'no-cors',headers:{'Content-Type':'text/plain'},
                                         body:JSON.stringify({fn:'home',args:[]})});
        log('Spot 공간 원위치 (책꽂이 · 책 · 로봇 처음 상태)');}
    catch(e){log('Spot 서버(:7910)에 닿지 않습니다',true);}
  };
  $('#spotReload').onclick=async()=>{
    const fr=$('#spotFrame');
    if(await spotAlive()){fr.setAttribute('src',SPOT_URL+'?t='+Date.now());$('#spotMsg').textContent='';}
    else $('#spotMsg').textContent='Spot 서버(:7910)가 꺼져 있습니다 — 뮤에서 bash ~/dusk_tpu/start_spot.sh';
  };

  /* 슬라이더 생성 — 로봇을 바꾸면 축 수가 달라지므로 다시 만든다 */
  function buildSliders(){
  const sl=$('#sliders');
  sl.innerHTML='';
  META.joints.forEach((j,i)=>{
    const d=document.createElement('div');d.className='sliderRow';
    if(j.dead)d.style.display='none';   // 모터 없는 고정 관절: 숨기되 순서·값(0)은 유지
    d.innerHTML=`<span class="jname">${j.label}</span>
      <input type="range" min="${j.min}" max="${j.max}" step="0.1" value="0">
      <span class="jval">0°</span>`;
    const r=d.querySelector('input');
    r.oninput=()=>d.querySelector('.jval').textContent=r.value+'°';
    r.onchange=()=>{HASH.sliders=null;
      call('set_sliders',selPose,...$$('#sliders input').map(x=>+x.value));};
    sl.append(d);
  });
  }
  buildSliders();
  window.rebuildSliders=async()=>{                 // 로봇 교체 후 호출
    META=await(await fetch('/api/meta')).json();
    buildSliders();
  };

  /* 정적 선택지 */
  opts($('#ftStep'),META.steps,'1');
  opts($('#speed'),META.speeds,'1.5x');
  opts($('#macroSlot'),META.macro_slots,META.macro_slots[0]);
  if($('#pegH')){$('#pegH').max=META.peg_h_max;$('#pegHV').value=$('#pegH').value;}

  /* ---- 자세 탭 ---- */
  $('#btnGripOpen').onclick=()=>call('pose_grip',selPose,true);
  $('#btnGripClose').onclick=()=>call('pose_grip',selPose,false);
  if($('#btnLvMinus')){                                       // 11-05: 그리퍼 수평 (실물 j4 영점)
    const lv=async c=>{const j=await call('level_trim',c);if(j&&j.status)setTicker(String(j.status));};
    $('#btnLvMinus').onclick=()=>lv('-0.5');
    $('#btnLvPlus').onclick=()=>lv('+0.5');
    $('#btnLvReset').onclick=()=>{if(confirm('이 팔의 그리퍼 수평 보정을 0 으로 되돌릴까요?'))lv('reset');};
  }
  $('#btnFtGo').onclick=()=>call('ft_goto',selPose);
  $('#btnFtMinus').onclick=()=>call('ft_step',selPose,ftJoint,$('#ftStep').value,-1);
  $('#btnFtPlus').onclick=()=>call('ft_step',selPose,ftJoint,$('#ftStep').value,1);
  $('#btnFtGOpen').onclick=()=>call('ft_grip',selPose,true);
  $('#btnFtGClose').onclick=()=>call('ft_grip',selPose,false);
  $('#btnFtApply').onclick=()=>call('ft_apply',selPose);
  $('#btnFtReset').onclick=()=>call('ft_reset',selPose);
  $('#btnFtSave').onclick=()=>call('ft_save',selPose);
  // 📷 카메라 동기화 — 2026-10-20 사용 중지 (index.html 의 그룹도 주석). 다시 쓰려면 true
  const CAM_UI=false;
  if(CAM_UI){
  // 📷 카메라 동기화 — 상태 확인은 로그를 어지럽히지 않게 조용히 부른다
  const camQuiet=async(fn,...args)=>{try{const r=await fetch('/api/call',{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify({fn,args})});return await r.json();}catch(e){return null;}};
  const camShow=j=>{if(j&&j.status)$('#camStat').textContent=j.status;};
  $('#btnCamLoad').onclick=async()=>{
    camShow(await call('cam_load',$('#camPath').value));
    for(let i=0;i<120;i++){                          // 분석이 끝날 때까지 (최대 2분)
      await new Promise(r=>setTimeout(r,1000));
      const j=await camQuiet('cam_status'); camShow(j);
      if(j&&j.status&&!/분석 중/.test(j.status))break;
    }
  };
  let camPlaying=true;
  $('#btnCamPlay').onclick=async()=>{camPlaying=!camPlaying;camShow(await camQuiet('cam_ctl',camPlaying?'play':'pause'));};
  $('#btnCamFp').onclick=async()=>camShow(await camQuiet('cam_ctl','fp'));
  $('#btnCamRot').onclick=async()=>camShow(await camQuiet('cam_ctl','rot'));
  $('#btnCamAuto').onclick=async()=>camShow(await camQuiet('cam_ctl','auto'));
  $('#btnCamBg').onclick=async()=>camShow(await camQuiet('cam_ctl','view'));
  $('#btnCamPlates').onclick=async()=>{const j=await call('cam_fit_plates');camShow(j);};
  setInterval(async()=>{                              // 자동 반영 소식을 2초마다 보여준다
    if(!$('#tab-link')||!$('#tab-link').classList.contains('on'))return;
    const j=await camQuiet('cam_status'); if(j&&j.status&&!/영상 없음/.test(j.status))camShow(j);
  },2000);
  document.querySelectorAll('.camMv').forEach(b=>b.onclick=async()=>camShow(await camQuiet('cam_ctl',b.dataset.c)));
  }
  // 📱 휴대폰 카메라 (실시간, 2026-10-22) — 휴대폰이 /api/live/frame 으로 보내는 동안 카메라 위치가 3D 에
  if($('#liveStat')){
    const lq=async(fn,...args)=>{try{const r=await fetch('/api/call',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({fn,args})});return await r.json();}catch(e){return null;}};
    const ls=j=>{if(j&&j.status)$('#liveStat').textContent=j.status;};
    if($('#phoneUrls')&&META&&META.phone_urls)
      $('#phoneUrls').innerHTML=META.phone_urls.map(u=>`<div>${u}</div>`).join('');
    document.querySelectorAll('.liveMv').forEach(b=>b.onclick=async()=>ls(await lq('live_ctl',b.dataset.c)));
    /* 🗺 보정 지도 (11-04) */
    window.calMapStat=async(c='status')=>{const j=await lq('calmap',c);
      if(j&&j.status&&$('#calMapStat'))$('#calMapStat').textContent=j.status;};
    if($('#btnCalMap')){$('#btnCalMap').onclick=()=>window.calMapStat(null);window.calMapStat();}
    if($('#btnCalOk'))$('#btnCalOk').onclick=async()=>{const j=await lq('calmap_ok');
      if(j&&j.status){log(j.status);setTicker(j.status);} window.calMapStat();};
    /* 🖥 판다 USB 웹캠 (11-04) — 휴대폰과 같은 길(/api/live/frame 처리기)로 서버가 직접 넣는다 */
    const pc=async c=>{try{const j=await (await fetch('/api/panda_cam/'+c,{method:'POST'})).json();
      if(j&&j.status)$('#pandaCamStat').textContent=j.status;}catch(e){}};
    if($('#btnPandaCam')){$('#btnPandaCam').onclick=()=>pc('toggle');pc('status');}
    setInterval(async()=>{                     // 연동 탭이 열려 있을 때만 2초마다 상태
      if(!$('#tab-link')||!$('#tab-link').classList.contains('on'))return;
      ls(await lq('live_status'));
      pc('status');
    },2000);
  }
  $('#btnCalOff').onclick=()=>call('cal_torque_off',
    $('#calRobot .on')?.textContent||META.cal_labels[0]);
  $('#btnCalCap').onclick=()=>call('cal_capture',
    $('#calRobot .on')?.textContent||META.cal_labels[0]);
  $('#btnDup').onclick=()=>call('dup',selPose,$('#newName').value);
  $('#btnDel').onclick=()=>{
    if(poseDelOk(selPose)){
      call('del',selPose);selPose='base';HASH.poses=null;}
  };
  $('#btnSave').onclick=()=>call('save');
  $('#btnLoad').onclick=()=>call('load');
  $('#btnPath').onclick=()=>call('add_path',$('#pathIn').value);
  $('#upDir').onchange=e=>upload(e.target.files,true);
  $('#upStl').onchange=e=>upload(e.target.files,false);

  /* ---- ② 시뮬레이션: 블록 팔레트 드래그 앤 드랍 ---- */
  pipelineDnD();
  $('#btnBlkUp').onclick=()=>sel&&call('seq_move',sel,-1).then(()=>{sel=null;});
  $('#btnBlkDn').onclick=()=>sel&&call('seq_move',sel,1).then(()=>{sel=null;});
  $('#btnBlkDel').onclick=()=>sel&&call('seq_del',sel).then(()=>{sel=null;refresh();});
  $('#btnBlkClear').onclick=async()=>{                 // 블록 일괄 삭제
    const n=(ST?.seq||[]).length;
    if(!n){setTicker('파이프라인이 이미 비어 있습니다');return;}
    if(!confirm(`블록 ${n}개를 전부 지울까요? (Ctrl+Z 로 되돌릴 수 있습니다)`))return;
    sel=null;HASH.seq=null;await call('seq_clear');
  };
  /* (숨긴 UI 의 바인딩 - 요소가 있을 때만: 세그 설정/무시 장애물) */
  if($('#segMode')){
    $('#segMode').onchange=()=>call('seg_mode',sel,$('#segMode').value);
    $('#segAx').onchange=()=>call('seg_ax',sel,$('#segAx').value);
    $('#segRot').onchange=()=>call('seg_rot',sel,$('#segRot').value);
    $('#btnRec').onclick=()=>call('rec_toggle',sel);
    $('#btnIgnSave').onclick=()=>call('ign_save',sel,
      $$('#ignList input:checked').map(i=>i.value)).then(fetchMap);
  }
  /* 로봇 선택 — 앱 재시작 없이 바로 바뀐다 */
  async function loadRobots(){
    const sel=$('#robotSel'); if(!sel)return;
    const j=await call('robot_list');
    const rs=((j&&j.outputs&&j.outputs[0])||{}).robots||[];
    sel.innerHTML=rs.map(r=>
      `<option value="${esc(r.name)}"${r.active?' selected':''}>`
      +`${esc(r.name)}${r.active?'  (지금)':''}`
      +`</option>`).join('');
    $('#robotNote').textContent=rs.length
      ? '고르면 바로 바뀝니다 (자세는 base 로 시작)'
      : 'urdf 폴더에 로봇이 없습니다';
  }
  /* 11-05 (사용자): 로봇은 SO-ARM (STS3215) 하나 — 다른 로봇이 골라져 있으면 자동으로 바꾼다 (칸은 숨김) */
  const ROBOT_ONLY='SO-ARM (STS3215)';
  (async()=>{
    await loadRobots();
    const sel=$('#robotSel');
    const has=sel&&[...sel.options].some(o=>o.value===ROBOT_ONLY);
    const cur=sel&&[...sel.options].find(o=>o.selected);
    if(has&&cur&&cur.value!==ROBOT_ONLY){sel.value=ROBOT_ONLY;$('#btnRobotPick').click();}
  })();
  $('#btnRobotPick').onclick=async()=>{
    const n=$('#robotSel').value; if(!n)return;
    const b=$('#btnRobotPick'); b.disabled=true; b.textContent='바꾸는 중…';
    try{
      await call('robot_pick',n);
      await loadRobots();          // '(지금)' 표시 갱신
      if(window.rebuildSliders)await window.rebuildSliders();   // 축 수 반영
      if(typeof fetchMap==='function')await fetchMap();
      const v=$('#viewer');        // 3D 를 새 로봇으로 다시 그린다
      if(v)v.contentWindow.location.reload();
    }finally{ b.disabled=false; b.textContent='이 로봇으로'; }
  };

  $('#btnRStat').onclick=async()=>{
    const r=await call('robot_status');
    if(r?.outputs)$('#robotStat').textContent=r.outputs;
  };
  const runArgs=()=>['간편 이동',META.modes[0],'x-z-y',
    $('#speed').value,$('#loop').checked,false];
  /* 11-05 (사용자): 미리보기가 느리다 — 3D 미리보기에만 배속을 곱한다 (실물 실행은 runArgs 그대로) */
  const pvArgs=()=>['간편 이동',META.modes[0],'x-z-y',pvSpeed(),$('#loop').checked,false];
  {
    const pb=$('#pvBoost');
    if(pb){
      try{const v=localStorage.getItem('pvBoost');if(v)pb.value=v;}catch(e){}
      pb.onchange=()=>{try{localStorage.setItem('pvBoost',pb.value);}catch(e){}};
    }
  }
  /* ---------- 말로 만들기: 텍스트 + 마이크(브라우저 음성 인식) ---------- */
  async function sayCmd(t){
    t=(t||'').trim(); if(!t)return;
    log('🗣 '+t); setTicker('명령: '+t);
    BLK_ANIM.prev=(ST?.seq||[]).filter(x=>x.kind!=='move').map(blkKey);BLK_ANIM.arm=Date.now()+60000;
    const j=await call('say',t);
    BLK_ANIM.arm=Date.now()+4000;                 // 결과가 온 뒤 다음 화면 갱신에서 새 블록을 찾는다
    if(!j)return;
    const r=(j.outputs&&j.outputs[0])||{};
    if(r.speed){const s=$('#speed');if(s)s.value=r.speed;}   // "속도 느리게"
    if(r.run){call('run',...runArgs());return;}              // "실물 실행 확인"
    if(r.preview)call('preview',...pvArgs());    // 말하는 즉시 3D 미리보기 (미리보기 배속)
  }
  {
    const inp=$('#sayTxt');
    /* AI 모델 선택 (RACHEL/Gemma/규칙) — 서버가 다음 명령부터 바로 적용 */
    const am=$('#aiModel');
    if(am)am.onchange=async()=>{
      const r=await call('llm_engine',am.value);
      const t=r&&(r.outputs&&r.outputs[0]||r.outputs);
      if(t)setTicker(String(t));
    };
    /* 11-05 (사용자): 말로 만들기는 더블 레이첼 (음성 인식만 Spark) — 화면을 열 때 서버도 맞춘다 */
    if(am){am.value='rachel2';call('llm_engine','rachel2');}
    $('#btnSay').onclick=()=>{sayCmd(inp.value);inp.value='';};
    inp.addEventListener('keydown',e=>{
      if(e.key==='Enter'){sayCmd(inp.value);inp.value='';}});
    const SR=window.SpeechRecognition||window.webkitSpeechRecognition;
    const mic=$('#btnMic');
    let sttOK=null;                 // Spark Whisper 사용 가능 여부
    /* ── 음성 인식 엔진 고르기 ──────────────────────────────
       자동 / Spark / 이 PC(Moonshine) 중에서 고른다.
       고른 값은 서버에 알려주고, 서버가 그대로 따른다. */
    const sttSel=$('#sttEngine');
    if(sttSel){
      fetch('/api/stt_engine').then(r=>r.json()).then(j=>{
        /* ★ 서버가 쓰고 있는 값을 그대로 보여준다.
             목록에 없는 값이면(옛 화면이 캐시된 경우) 억지로 바꾸지 않는다 —
             전에 이것 때문에 서버의 'both' 설정이 'auto' 로 덮어써졌다. */
        if(j&&j.engine){
          const has=[...sttSel.options].some(o=>o.value===j.engine);
          if(has)sttSel.value=j.engine;
          else setTicker('음성 엔진: '+j.engine+' (화면 갱신이 필요합니다)');
        }
        if(j&&!j.have_moonshine){
          const o=sttSel.querySelector('option[value=moonshine]');
          if(o){o.disabled=true;o.textContent='이 PC · WHISPER DUSK (모델 없음)';}
        }
      }).catch(()=>{});
      sttSel.addEventListener('change',async()=>{
        try{
          const r=await fetch('/api/stt_engine',{method:'POST',
            headers:{'Content-Type':'application/json'},
            body:JSON.stringify({engine:sttSel.value})});
          const j=await r.json();
          if(!j.ok){setTicker('음성 엔진 변경 실패: '+(j.error||''));return;}
          setTicker('음성 인식: '+sttSel.options[sttSel.selectedIndex].textContent);
        }catch(e){setTicker('음성 엔진 변경 실패: '+e);}
      });
    }
    fetch('/api/stt_ping').then(r=>r.json()).then(j=>{
      sttOK=!!j.ok;
      if(sttOK)mic.textContent='🎤 누르고 말하기 (Spark Whisper)';
    }).catch(()=>{sttOK=false;});
    const micIdle=()=>{mic.classList.remove('danger');
      mic.textContent=sttOK?'🎤 누르고 말하기 (Spark Whisper)':'🎤 누르고 말하기';};
    /* --- 방식 A: Spark Whisper (녹음 -> WAV -> /api/stt) --- */
    let W=null,listening=false;
    /* --- 방식 A-2: 🖥 판다 마이크 (11-04) ---
       브라우저 마이크가 안 열리면(다른 PC 에서 http 로 연 경우, 파이어폭스가 16kHz 녹음을
       못 여는 경우) 판다에 꽂은 USB 마이크로 서버가 직접 녹음한다. 미리보기·인식은 그대로. */
    async function pStart(why){
      try{
        const j=await (await fetch('/api/panda_mic/start',{method:'POST'})).json();
        if(!j.ok){setTicker('마이크 사용 불가: '+why+' / 판다 마이크: '+(j.error||''));return;}
      }catch(e){setTicker('마이크 사용 불가: '+why+' / 판다 마이크: '+e);return;}
      let busyPv=false;
      const timer=setInterval(async()=>{
        if(busyPv||!listening)return;
        busyPv=true;
        try{
          const b=await (await fetch('/api/panda_mic/peek')).blob();
          if(b.size<16000)return;              // 0.5초 안 됨
          const r=await sttPost(b,true);
          if(listening&&r.ok&&r.text){inp.value=r.text;setTicker('🎤 '+r.text);}
        }catch(e){}
        finally{busyPv=false;}
      },1000);
      W={panda:true,timer};listening=true;
      mic.classList.add('danger');
      mic.textContent='🔴 판다 마이크로 듣는 중... (다 말하면 다시 누르세요)';
    }
    async function wStart(){
      let st=null;
      try{
        st=await navigator.mediaDevices.getUserMedia({audio:true});
        const ac=new AudioContext({sampleRate:16000});
        const src=ac.createMediaStreamSource(st);
        const proc=ac.createScriptProcessor(4096,1,1);
        const chunks=[];
        /* 녹음만 모은다. 끝내는 건 사용자가 버튼을 다시 눌러서.
           (무음 자동종료는 말 중간의 뜸에 잘려 나가 오히려 불편했다 —
            사용자 요청으로 예전 방식으로 되돌림) */
        proc.onaudioprocess=e=>
          chunks.push(new Float32Array(e.inputBuffer.getChannelData(0)));
        src.connect(proc);proc.connect(ac.destination);

        /* ★ 실시간 미리보기 ★
           whisper.cpp 서버는 스트리밍을 지원하지 않는다(공식 이슈 #2038).
           그래서 '지금까지 녹음한 것' 을 1초마다 통째로 보내 글자를 받아
           화면에 갱신한다. 진짜 스트리밍은 아니지만 보기엔 같다.
           - 실행은 버튼을 다시 눌렀을 때 '전체' 를 다시 변환해서 한다
             (미리보기는 참고용이라 조금 틀려도 상관없다)
           - 앞선 요청이 아직 안 왔으면 건너뛴다 (Spark 에 쌓이지 않게) */
        let busyPv=false;
        const timer=setInterval(async()=>{
          if(busyPv||!listening||chunks.length===0)return;
          busyPv=true;
          try{
            /* 말하는 중 미리보기 — 1초마다 돈다.
               ★ preview=1 을 붙여 비교 모드라도 한 엔진만 돌게 한다.
                 안 그러면 매초 두 엔진이 다 돌아 느려지고,
                 로그도 도배된다. 비교는 '다 말한 뒤' 한 번만 한다. */
            const r=await sttPost(toWav(chunks),true);
            if(listening&&r.ok&&r.text){
              inp.value=r.text;                 // 말하는 대로 글자가 채워진다
              setTicker('🎤 '+r.text);
            }
          }catch(e){}
          finally{ busyPv=false; }
        },1000);

        W={st,ac,proc,chunks,timer};listening=true;
        mic.classList.add('danger');
        mic.textContent='🔴 듣는 중... (다 말하면 다시 누르세요)';
      }catch(e){
        if(st)st.getTracks().forEach(t=>t.stop());
        await pStart(String(e));
      }
    }
    /* 녹음 조각들 -> WAV 한 덩어리 (16kHz 모노 16bit) */
    function toWav(chunks){
      const n=chunks.reduce((a,c)=>a+c.length,0);
      const pcm=new Int16Array(n);let o=0;
      for(const c of chunks)for(let i=0;i<c.length;i++)
        pcm[o++]=Math.max(-32768,Math.min(32767,c[i]*32767|0));
      const h=new ArrayBuffer(44),v=new DataView(h);
      const ws=(p,s)=>{for(let i=0;i<s.length;i++)v.setUint8(p+i,s.charCodeAt(i));};
      ws(0,'RIFF');v.setUint32(4,36+pcm.byteLength,true);ws(8,'WAVEfmt ');
      v.setUint32(16,16,true);v.setUint16(20,1,true);v.setUint16(22,1,true);
      v.setUint32(24,16000,true);v.setUint32(28,32000,true);
      v.setUint16(32,2,true);v.setUint16(34,16,true);
      ws(36,'data');v.setUint32(40,pcm.byteLength,true);
      return new Blob([h,pcm.buffer]);
    }
    const sttPost=async(blob,preview)=>
      (await (await fetch('/api/stt'+(preview?'?preview=1':''),{method:'POST',
        headers:{'Content-Type':'audio/wav'},body:blob})).json());

    async function wStop(){
      if(!W)return;                       // 자동/수동이 겹쳐도 한 번만
      const w=W;W=null;listening=false;
      if(w.timer)clearInterval(w.timer);
      mic.textContent = (sttSel && sttSel.value==='moonshine')
        ? '⏳ W.DUSK 인식 중...' : '⏳ 인식 중...';
      try{
        let blob;
        if(w.panda){
          blob=await (await fetch('/api/panda_mic/stop',{method:'POST'})).blob();
        }else{
          const {st,ac,proc,chunks}=w;
          proc.onaudioprocess=null;
          proc.disconnect();st.getTracks().forEach(t=>t.stop());ac.close();
          blob=toWav(chunks);
        }
        const r=await sttPost(blob);
        micIdle();
        if(r.ok&&r.text){inp.value=r.text;
          /* 어느 엔진이 받아썼는지 같이 보여준다 — 비교해 보시라고 */
          const eng = r.engine==='moonshine' ? 'W.DUSK'
                    : r.engine==='spark' ? 'Spark' : '';
          /* 비교 모드면 둘 다 '전체 로그' 에 남긴다.
             ★ setTicker 는 위쪽 띠 한 줄을 덮어쓸 뿐이라, 여러 줄을
               연달아 부르면 마지막 것만 남고 로그에는 안 쌓인다.
               그래서 log() 를 쓴다 — 이게 전체 로그에 들어간다. */
          if(r.fixed&&!r.compare)
            log('🔧 W.DUSK 다듬 ('+r.fixed.ms+'ms): '
                +r.fixed.before+'  ->  '+r.fixed.after);
          if(r.compare){
            const c=r.compare;
            log('🔍 DGX Spark ('+c.spark_ms+'ms): '
              +(c.spark!=null?c.spark:'실패 — '+(c.spark_err||'')));
            if(c.fixed){
              log('🔍 W.DUSK 원본 ('+c.moonshine_ms+'ms): '+c.moonshine_raw);
              log('🔧 W.DUSK 다듬 ('+c.fixed.ms+'ms): '+c.moonshine);
            }else{
              log('🔍 WHISPER DUSK ('+c.moonshine_ms+'ms): '
                +(c.moonshine!=null?c.moonshine:'실패 — '+(c.moonshine_err||'')));
            }
            if(c.spark&&c.moonshine)
              log('🔍 두 결과가 '+(c.spark===c.moonshine?'같습니다':'다릅니다'));
          }
          setTicker('🎤 '+r.text+(eng?'  ['+eng+']':''));
          sayCmd(r.text);inp.value='';}
        else setTicker('음성 인식 실패: '+(r.error||'말이 안 들렸습니다'));
      }catch(e){micIdle();setTicker('음성 인식 오류: '+e);}
    }
    /* --- 방식 B (대체): 브라우저 내장 인식 (실시간 중간 표시) --- */
    let rec=null,srOn=false;
    function srStart(){
      rec=new SR();rec.lang='ko-KR';rec.interimResults=true;
      let fin='';
      rec.onstart=()=>{srOn=true;mic.classList.add('danger');
        mic.textContent='🔴 듣는 중... (다시 누르면 끝)';};
      rec.onresult=e=>{
        let s='';
        for(const r of e.results){s+=r[0].transcript;if(r.isFinal)fin=s;}
        inp.value=s;setTicker('🎤 '+s);
      };
      rec.onerror=e=>setTicker('음성 인식 오류: '+e.error+
        (e.error==='not-allowed'?' — 브라우저 마이크 권한을 허용하세요':''));
      rec.onend=()=>{srOn=false;micIdle();
        const s=(fin||inp.value||'').trim();
        if(s){sayCmd(s);inp.value='';}};
      rec.start();
    }
    mic.onclick=()=>{
      if(sttOK){listening?wStop():wStart();return;}     // Spark Whisper 우선
      if(!SR){setTicker('음성 인식 불가 — Spark 연결 또는 크롬 필요');return;}
      if(srOn){rec.stop();return;}
      srStart();
    };
  }
  /* 블록을 자세로 등록 -> ③연동 탭의 기존 미세조정으로 보정 */
  $('#btnBlkPose').onclick=async()=>{
    if(!sel){setTicker('먼저 파이프라인에서 블록을 클릭해 고르세요');return;}
    const j=await call('blk_to_pose',sel);
    if(j&&j.status&&!/없습니다|불가|고르세요/.test(j.status)){
      const m=j.status.match(/'([^']+)'/);
      if(m){selPose=m[1];HASH.poses=null;HASH.sliders=null;await refresh();}
      setTicker('자세 등록 완료 → ③연동 탭에서 미세조정하세요');
    }
  };
  /* 타임바: 끌면 그 블록까지 재생하고 멈춘다 */
  {
    const rg=$('#tbRange');
    rg.oninput=()=>{
      tbDragging=true;
      const i=+rg.value, lab=(ST?.seq?.[i]?.label||'').replace(/^[■◆◈➜▼▲]\s*/,'');
      $('#tbNow').textContent=`⏩ ${i+1}번 블록까지 — ${lab}  (놓으면 이동)`;
    };
    // 놓는 즉시 그 블록의 자세로 '점프' (처음부터 재생하지 않음)
    /* ★ tbDragging 을 반드시 풀어 준다 ★
       oninput 에서 true 로 켜고 onchange 에서만 껐는데, 블록이 없거나
       onchange 가 안 뜨면(크롬은 슬라이더에서 change 를 안 줄 때가 있다)
       영영 true 로 남아 타임바가 얼어붙었다(= 되감기가 안 되던 이유).
       마우스/터치를 떼는 순간에도 확실히 풀어 준다. */
    const _seek=async()=>{
      tbDragging=false;
      if(!ST||!ST.seq||!ST.seq.length)return;
      await call('seq_seek',+rg.value);
    };
    rg.onchange=_seek;
    rg.addEventListener('pointerup',_seek);
    rg.addEventListener('mouseup',_seek);
    rg.addEventListener('touchend',_seek);
    rg.addEventListener('keyup',_seek);
    rg.addEventListener('blur',()=>{tbDragging=false;});
  }
  $('#btnPreview').onclick=()=>{                      // ② 시뮬레이션
    const sb=ST?.seek_blk;
    if(sb>=0){                                       // 타임바 지점부터 이어재생
      setTicker(`${sb+1}번 블록부터 재생합니다`);
      return call('preview_from',sb,pvSpeed());
    }
    return call('preview',...pvArgs());
  };
  $('#btnPreview2').onclick=()=>call('preview',...pvArgs());  // ③ 연동
  $('#btnRun').onclick=()=>call('run',...runArgs());
  $('#btnStop2').onclick=()=>call('stop');
  $('#btnPause').onclick=()=>call('pause');     // 11-06 일시 정지 / 계속
  $('#btnStopSim').onclick=()=>call('stop');
  $('#btnToff').onclick=()=>call('torque_off');
  // 원위치 = 로봇 기본 자세 + 물체는 [불러오기] 하면 나오는 그 배치 위치로 (2026-10-20)
  const goHome=()=>{HASH.bricks=null;return call('home_zero',($('#fpSel')&&$('#fpSel').value)||'');};
  $('#btnHomeZero').onclick=()=>document.querySelector('main').classList.contains('spot')&&window.spotHome?window.spotHome():goHome();   // 11-06: Spot 화면이면 Spot 공간 원위치
  if($('#joyHome'))$('#joyHome').onclick=goHome;
  $('#btnEstop').onclick=()=>call('stop');
  $('#btnMRec').onclick=()=>call('macro_rec',$('#macroSlot').value);
  $('#btnMClr').onclick=()=>call('macro_clear',$('#macroSlot').value);
  /* (숨긴 UI 의 바인딩 - 요소가 있을 때만: 위에서 본 맵/간편 설정) */
  if($('#mapBox')){
    $('#btnMapUndo').onclick=()=>call('map_undo',sel).then(fetchMap);
    $('#btnMapClear').onclick=()=>call('map_clear',sel).then(fetchMap);
    $('#btnMapRef').onclick=fetchMap;
    $('#btnPathSave').onclick=()=>call('path_save',sel).then(fetchMap);
    $('#btnPathDel').onclick=()=>call('path_del',sel).then(fetchMap);
    $('#spClear').oninput=()=>$('#spClearV').value=$('#spClear').value;
    $('#spExtra').oninput=()=>$('#spExtraV').value=$('#spExtra').value;
    $('#spClear').onchange=()=>simpleSet(null);
    $('#spExtra').onchange=()=>simpleSet(null);
  }
  /* 맵 클릭 → 숨은 input (#mapxy) → 서버 */
  $('#mapxy input').addEventListener('input',e=>{
    call('map_xy',sel,e.target.value).then(fetchMap);
  });

  /* ---- 공장 꾸미기 ---- */
  /* 바닥 확장 모드는 3D 더블클릭으로 켜고 끔 (체크박스 제거됨) */
  $('#btnPlateDel').onclick=()=>call('plate_remove',$('#plateSel').value);
  $('#btnPlateRef').onclick=()=>call('plate_refresh');
  $('#robotMove').onchange=e=>call('robot_move_mode',e.target.checked);
  $('#btnRobotHome').onclick=()=>call('robot_home');
  /* 공장 저장/불러오기 */
  const fpRefresh=()=>{HASH.env=null;HASH.bricks=null;HASH.rpos=null;};
  $('#btnFpSave').onclick=async()=>{                 // 11-05: 비우면 지금 배치에 덮어쓰기
    await call('factory_save',$('#fpName').value.trim());
    $('#fpName').value='';
  };
  $('#btnFpLoad').onclick=()=>{fpRefresh();call('factory_load',$('#fpSel').value);};
  $('#btnFpDel').onclick=()=>{
    if(confirm(`배치 '${$('#fpSel').value}' 를 삭제할까요?`))
      call('factory_del',$('#fpSel').value);
  };
  $('#btnFpReset').onclick=()=>{
    if(confirm('공장을 비울까요? (플레이트만 남음 · Ctrl+Z 로 되돌리기 가능)')){
      fpRefresh();call('factory_reset');
    }
  };

  /* ---- 공간 좌표(목심) UI - 현재 숨김, 요소 있을 때만 ---- */
  if($('#pegView')){
    $('#pegView').onchange=e=>call('peg_view',e.target.checked,+$('#pegH').value);
    $('#pegH').oninput=()=>$('#pegHV').value=$('#pegH').value;
    $('#pegH').onchange=()=>{if($('#pegView').checked)call('peg_height',+$('#pegH').value);};
    $('#pegSel').onchange=()=>call('peg_pick',$('#pegSel').value);
    $('#btnPegGo').onclick=()=>call('peg_goto',$('#pegSel').value,+$('#pegH').value);
    $('#btnPegPose').onclick=()=>call('peg_save_pose',
      $('#pegSel').value,+$('#pegH').value,$('#pegPName').value);
  }
  if($('#btnPlace')){
    $('#btnPlace').onclick=()=>call('place',$('#brickSel').value,$('#dockSel').value);
    $('#btnGrasp').onclick=()=>call('grasp',$('#brickSel').value);
    $('#btnRelease').onclick=()=>call('release',$('#dockSel').value);
    $('#btnGraspAdd').onclick=()=>call('seq_grasp',$('#brickSel').value);
    $('#btnReleaseAdd').onclick=()=>call('seq_release',$('#dockSel').value);
  }
  /* 이름·색: 3D 좌클릭 = 대상 선택, 입력/색 변경 = 즉시 적용 (버튼 없음) */
  const nmApply=()=>{
    if(!nmSel)return;
    HASH.env=null;HASH.bricks=null;
    call('set_name',nmSel,$('#nmText').value,$('#nmColor').value);
  };
  $('#nmText').onchange=nmApply;          // 엔터/포커스 아웃 = 적용
  $('#nmColor').onchange=nmApply;
  addEventListener('message',e=>{
    if(e.data&&('calpose' in e.data)){          // 🗺 보정 지도 점 클릭 (11-04) → ③연동 미세조정으로
      if(e.data.armmsg)log(e.data.armmsg);
      const nm=e.data.calpose;
      if(nm){
        $('#tabs button[data-tab="link"]')?.click();
        selPose=nm;HASH.poses=null;HASH.sliders=null;
        call('select_pose',nm).then(()=>refresh());
        window.calMapStat?.();
      }
      return;
    }
    if(e.data&&e.data.armmsg){log(e.data.armmsg);HASH.palette=null;return;}   // 3D 에서 팔 우클릭 → 옮기기 (11-03)
    const id=e.data&&e.data.armsel;
    if(!id)return;
    $('#tabs button[data-tab="factory"]')?.click();   // ① 공장 꾸미기 탭으로
    nmSel=id;
    $('#nmCur').textContent=shortNm(id,ST?.names||{});
    $('#nmText').disabled=false;$('#nmColor').disabled=false;
    $('#nmText').value=ST?.names?.[id]?.name||'';
    if(ST?.names?.[id]?.color)$('#nmColor').value=ST.names[id].color;
    const g=$('#nmCur').closest('.group');
    if(g){g.scrollIntoView({behavior:'smooth',block:'center'});
      g.classList.add('flash');setTimeout(()=>g.classList.remove('flash'),1200);}
    $('#nmText').focus();
  });

  /* 로봇 자리 추천 (도달 안 되는 블록이 있을 때) */
  $('#btnSuggest').onclick=async()=>{
    const r=await call('robot_suggest');
    $('#btnSuggestGo').classList.toggle('hidden',
      !(r&&r.status&&r.status.includes('추천 자리')));
  };
  $('#btnSuggestGo').onclick=async()=>{
    HASH.seq=null;HASH.rpos=null;
    await call('robot_apply_suggest');
    $('#btnSuggestGo').classList.add('hidden');
  };

  /* Ctrl+Z = 실행취소 (공장 배치·삭제·복제 + 블록 편집) */
  addEventListener('keydown',e=>{
    if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'
       &&!/INPUT|TEXTAREA|SELECT/.test(document.activeElement?.tagName||'')){
      e.preventDefault();HASH.seq=null;HASH.env=null;call('undo');
    }
  });

  /* 첫 로드 + 주기 폴링 */
  await refresh();fetchMap();
  log('ARM STUDIO 준비 완료');
  setInterval(()=>{if(!document.hidden)refresh();},2000);

  /* 조종표 조각을 미리 받아둔다.
     예전에는 조이스틱을 처음 밀 때 받아서, 그 몇 초 동안은 표가 없어
     서버 IK 로 움직였다 -> "처음에는 안 부드럽고 하다 보면 부드러워짐".
     이제 이진이라 6ms 면 되므로 시작할 때 받아두면 처음부터 부드럽다. */
  if(window._joyPreload)window._joyPreload();   // 조이콘 쪽이 등록해 둔다
}

async function upload(files,isDir){
  if(!files.length)return;
  const fd=new FormData();const paths=[];
  [...files].forEach(f=>{fd.append('files',f);
    paths.push(f.webkitRelativePath||f.name);});
  fd.append('paths',JSON.stringify(paths));
  log(`업로드 중... (${files.length}개 파일)`);
  try{
    const j=await(await fetch('/api/upload',{method:'POST',body:fd})).json();
    if(j.job)await pollJob();else log(j.status||j.error||'업로드 완료');
    await refresh();
  }catch(e){log('업로드 실패: '+e,true);}
}

boot();

/* ================= ④ 조이콘 조종 =================
   브라우저 게임패드 API 로 스틱을 읽는다.
   -> 연결 버튼도, 스틱 찾기도 필요 없다. 아무 버튼이나 누르면 잡힌다.
   (예전엔 IMU 기울기 때문에 WebHID 로 원시 바이트를 읽었는데,
    스틱 바이트 위치를 찾아야 하고 값 환산도 틀리기 쉬웠다.
    기울기 방식을 뺐으므로 단순한 게임패드 API 로 되돌렸다 — 2026-08-24)
   물컵 규칙(수평 유지)은 서버의 수평 유지 IK 가 보장한다. */
(function(){
  const el=s=>document.querySelector(s);
  if(!el('#joyDiag'))return;                  // 탭이 없으면 아무것도 안 함
  const J={on:false, spd:0.25, dead:15, grip:null, bUp:7, bDn:1};
  const NINTENDO=/057e|joy-?con|nintendo|switch|pro controller/i;
  function pad(){
    if(!navigator.getGamepads)return null;
    const all=[...navigator.getGamepads()].filter(Boolean);
    return all.find(g=>NINTENDO.test(g.id||''))||all[0]||null;
  }
  addEventListener('gamepadconnected',e=>{
    if(el('#joyStat'))el('#joyStat').textContent='연결됨 — '+e.gamepad.id;
  });
  addEventListener('gamepaddisconnected',()=>{
    if(el('#joyStat'))el('#joyStat').textContent='끊김 — 아무 버튼이나 누르세요';
  });

  /* 시작 버튼 없이 — 스틱/버튼을 처음 건드리면 자동으로 시작한다.
     (joy_start 는 '지금 자세를 출발점으로 잡는' 서버 준비 단계라 필요하다) */
  let starting=false;
  async function autoStart(){
    if(J.on||starting)return;
    starting=true;
    try{
      await call('joy_start'); J.on=true;
      await loadChunk();          // 조각을 받으면서 현재 위치도 함께 받는다
    }
    finally{ starting=false; }
  }
  if(el('#joyStop'))el('#joyStop').onclick=async()=>{
    J.on=false; await call('joy_stop');
    if(el('#joyStat'))el('#joyStat').textContent='정지됨 — 스틱을 움직이면 다시 시작';
  };
  if(el('#joyHome'))el('#joyHome').onclick=()=>call('home_zero');
  if(el('#headReload'))el('#headReload').onclick=async()=>{
    const r=await call('head_reload');
    setTicker((r&&r.status)||'표 다시 읽음');
    TBL.ready=false; TBL.cells.clear(); TBL.center=null;
    await loadChunk(...(TBL.pos||[]));
  };
  if(el('#joyTable'))el('#joyTable').onclick=()=>{
    setTicker('조종표 만드는 중… (한 번만, 몇 분 걸립니다)');
    call('joy_table').then(()=>setTicker('조종표 완료 — 이제 즉시 반응합니다'));
  };
  if(el('#joyHow'))el('#joyHow').innerHTML=
    '<b>스틱</b> = 앞뒤·좌우 · <b>위 버튼</b>=올리기 <b>아래 버튼</b>=내리기 · '
    +'<b>A</b>=그리퍼 열기 <b>B</b>=닫기';
  if(el('#joySpd'))el('#joySpd').oninput=e=>{
    /* 슬라이더가 이미 m/s 단위(0.05~0.8)다. 100 으로 나누면 100배 느려진다 */
    J.spd=+e.target.value; el('#joySpdV').textContent=J.spd.toFixed(2)+' m/s';};
  if(el('#joyDead'))el('#joyDead').oninput=e=>{
    J.dead=+e.target.value; el('#joyDeadV').textContent=J.dead+'%';};
  /* ---- VR 보기 (PSVR 입체 + 왜곡 보정) ----
     PSVR 렌즈는 가장자리를 바깥으로 휘게 만든다. 화면을 미리 안쪽으로
     휘어서 그려야 또렷하다. 사람마다 눈이 다르니 직접 맞추게 한다. */
  {
    // 모니터 복제로 쓰면 화면 하나가 그대로 보인다 -> 눈 간격은 안 쓴다.
    // 시야각 기본값은 평소 화면과 같게 (50~70).
    // fov 를 낮출수록 화면이 '당겨진다'(확대). 난시가 있으면 크게 볼수록 낫다.
    const DEF={ipd:0.064,k1:0.22,k2:0.06,fov:50,scale:1.0};
    let cfg={...DEF};
    try{ cfg={...DEF,...JSON.parse(localStorage.getItem('vrCfg')||'{}')}; }catch(e){}
    const vsend=m=>{
      const vf=document.querySelector('#viewer');
      if(vf&&vf.contentWindow)vf.contentWindow.postMessage({type:'vr',...m},'*');
    };
    const show=()=>{
      if(el('#vrK1V'))el('#vrK1V').textContent=(+cfg.k1).toFixed(2);
      if(el('#vrFovV'))el('#vrFovV').textContent=Math.round(cfg.fov)+'도';
      if(el('#vrScaleV'))el('#vrScaleV').textContent=(+cfg.scale).toFixed(2);
    };
    const put=()=>{
      try{ localStorage.setItem('vrCfg',JSON.stringify(cfg)); }catch(e){}
      vsend(cfg); show();
    };
    const bind=(id,key,f)=>{
      const e2=el(id); if(!e2)return;
      e2.value=cfg[key];
      e2.oninput=ev=>{ cfg[key]=f?f(ev.target.value):+ev.target.value; put(); };
    };
    bind('#vrK1','k1'); bind('#vrFov','fov'); bind('#vrScale','scale');
    show();
    let vrOn=false;
    if(el('#vrOn'))el('#vrOn').onclick=()=>{
      vrOn=!vrOn; vsend({on:vrOn,...cfg});
      el('#vrOn').classList.toggle('primary',!vrOn);
      setTicker(vrOn?'초점 보정 ON — 왜곡을 움직여 또렷하게 맞추세요'
                    :'초점 보정 OFF');
    };
    if(el('#vrReset'))el('#vrReset').onclick=()=>{
      cfg={...DEF};
      for(const [id,k] of [['#vrK1','k1'],
                           ['#vrFov','fov'],['#vrScale','scale']]){
        const e2=el(id); if(e2)e2.value=cfg[k];
      }
      put(); setTicker('VR 설정을 기본값으로 되돌렸습니다');
    };
  }
  if(el('#joyBUp'))el('#joyBUp').oninput=e=>{J.bUp=+e.target.value;};
  if(el('#joyBDn'))el('#joyBDn').oninput=e=>{J.bDn=+e.target.value;};

  /* ===== 조종표를 브라우저에 두고 직접 찾기 =====
     서버 왕복(40ms)이 곧 화면 속도가 되어 끊기던 것을 없앤다.
     주변 표를 미리 받아두고, 스틱을 밀면 브라우저가 즉시 칸을 찾아
     3D 를 그린다. 서버에는 결과 자세만 알려준다(느려도 화면은 안 밀림). */
  const TBL={ready:false,cells:new Map(),center:null,r:0,grid:0.01,nj:8,
             loading:false,pos:null,
             q:null};   // 마지막으로 보낸 관절값 — 외삽 안전장치의 기준
  function tkey(x,y,z){return x+','+y+','+z;}
  /* 앱이 뜨자마자 표 조각을 받아두기 위한 창구.
     boot() 는 이 블록 밖이라 loadChunk 를 직접 못 부른다. */
  window._joyPreload=()=>{ if(!TBL.ready&&!TBL.loading)loadChunk(); };
  // boot() 가 이미 지나갔을 수도 있다 -> 여기서도 한 번 (META 를 기다린다)
  (function _pre(){
    if(META&&META.viewer_port) window._joyPreload();
    else setTimeout(_pre,300);      // /api/meta 가 아직 안 왔다
  })();
  /* 조각은 '이진' 으로 받는다.
     JSON 4.1MB 는 전송에만 2.5초씩 걸려, 조각 경계를 지날 때마다
     그 시간만큼 조종이 버벅였다. 이진은 1/4 크기 + 파싱 즉시. */
  async function loadChunk(px,py,pz){
    if(TBL.loading)return;
    TBL.loading=true;
    try{
      // 뷰어와 같은 길(7900 중계)로 간다 — 포트 직결은 막힐 수 있다
      let url=`/viewer/chunk?r=25`;
      if(px!==undefined)
        url+=`&cx=${Math.round(px/TBL.grid)}&cy=${Math.round(py/TBL.grid)}`
            +`&cz=${Math.round(pz/TBL.grid)}`;
      const buf=await (await fetch(url)).arrayBuffer();
      if(buf.byteLength<32){TBL.loading=false;return;}
      const hd=new Int32Array(buf,0,6);
      const n=hd[0], nj=hd[1];
      const grid=new Float64Array(buf,24,1)[0];
      const keys=new Int16Array(buf,32,n*3);
      const vals=new Float32Array(buf,32+n*6,n*nj);
      TBL.cells.clear();
      for(let i=0;i<n;i++){
        TBL.cells.set(tkey(keys[i*3],keys[i*3+1],keys[i*3+2]),
                      Array.from(vals.subarray(i*nj,(i+1)*nj)));
      }
      TBL.center=[hd[2],hd[3],hd[4]]; TBL.r=hd[5];
      TBL.grid=grid; TBL.nj=nj; TBL.ready=true;
      if(!TBL.pos)TBL.pos=TBL.center.map(v=>v*grid);
      // 안전장치(외삽)는 '지금 자세'를 기준으로 삼는다. 조각을 받은 직후엔
      // 그 값이 없어 첫 조작이 통째로 막혔다 -> 중심 칸 자세로 채워둔다.
      if(!TBL.q&&TBL.center)
        TBL.q=TBL.cells.get(tkey(TBL.center[0],TBL.center[1],TBL.center[2]))||null;
      if(el('#joyDiag'))el('#joyDiag').textContent=
        `조종표 ${n.toLocaleString()}칸 준비됨 (즉시 반응)`;
    }catch(e){}
    TBL.loading=false;
  }
  function tblFind(x,y,z){
    if(!TBL.ready)return null;
    const kx=Math.round(x/TBL.grid),ky=Math.round(y/TBL.grid),kz=Math.round(z/TBL.grid);
    return TBL.cells.get(tkey(kx,ky,kz))||null;
  }
  /* ★ 이웃 8칸 사이 보간 ★
     칸(10mm)으로 반올림만 하면 자세가 '멈춤->10mm 점프' 를 반복해 뚝뚝
     끊긴다 (틱당 이동 8mm < 칸 10mm 라 칸이 안 바뀌는 틱이 생김).
     둘러싼 8칸의 관절값을 거리 비율로 섞으면 어느 위치든 연속이 된다.
     (닭 머리 표는 이웃끼리 씨앗을 이어받아 만들어 관절이 매끄럽게 변함) */
  function tblInterp(x,y,z){
    if(!TBL.ready)return null;
    const g=TBL.grid;
    const fx=x/g, fy=y/g, fz=z/g;
    const x0=Math.floor(fx), y0=Math.floor(fy), z0=Math.floor(fz);
    const tx=fx-x0, ty=fy-y0, tz=fz-z0;
    let acc=null, wsum=0;
    for(let dx=0;dx<=1;dx++)for(let dy=0;dy<=1;dy++)for(let dz=0;dz<=1;dz++){
      const c=TBL.cells.get(tkey(x0+dx,y0+dy,z0+dz));
      if(!c)continue;
      const w=(dx?tx:1-tx)*(dy?ty:1-ty)*(dz?tz:1-tz);
      if(w<=0)continue;
      if(!acc)acc=new Array(c.length).fill(0);
      for(let j=0;j<c.length;j++)acc[j]+=c[j]*w;
      wsum+=w;
    }
    if(!acc||wsum<0.5)return null;      // 절반 넘게 비면 신뢰 못 함
    for(let j=0;j<acc.length;j++)acc[j]/=wsum;
    return acc;
  }
  /* 서버가 표를 새로 읽으면(세대 변경) 로컬 조각도 새로 받는다 */
  let headGen=null;
  function headGenCheck(s){
    if(s.headgen===undefined)return;
    if(headGen!==null&&s.headgen!==headGen){
      TBL.ready=false; TBL.cells.clear(); TBL.center=null;
      loadChunk(...(TBL.pos||[]));
    }
    headGen=s.headgen;
  }
  window._headGenCheck=headGenCheck;      // render 에서 부를 수 있게

  /* 표 밖 한 칸 정도는 '기울기' 로 이어서 만든다.
     축마다 (있는 칸 - 그 이웃 칸) 차이를 구해 남은 거리만큼 더한다. */
  /* 표 밖일 때 '이어서 밀어주기'.

     ★ 반드시 지금 자세에서 이어져야 한다 ★
       표는 칸마다 따로 푼 것이라 이웃끼리 이어진다는 보장이 없다.
       실측: 바로 옆칸(10mm)인데 관절이 최대 220도 다른 자리가 있다
       (팔꿈치가 반대로 접힌 해). 그런 칸을 잡아 그 자세로 바꿔버리면
       팔이 조작하지 않은 방향으로 크게 휘둘린다.
       -> 지금 자세에서 많이 벗어나는 답은 아예 쓰지 않는다(= 안 움직인다).
     cur = 지금 관절값. 없으면 안전장치를 걸 수 없으므로 외삽하지 않는다. */
  const EXTRAP_MAX_DEG=6.0;      // 한 틱에 이만큼 넘게 변하면 버린다
  function tblExtrap(x,y,z,cur){
    if(!TBL.ready)return null;
    // 기준 자세가 아직 없으면(조각을 막 받은 직후) 중심 칸으로 세운다.
    // 예전엔 여기서 null 을 돌려줘 첫 조작이 조용히 막혔다.
    if(!cur&&TBL.center)
      cur=TBL.cells.get(tkey(TBL.center[0],TBL.center[1],TBL.center[2]))||null;
    if(!cur)return null;
    const g=TBL.grid;
    const kx=Math.round(x/g), ky=Math.round(y/g), kz=Math.round(z/g);
    // 가장 가까운 '있는 칸' 을 짧게 훑는다.
    // 단, 지금 자세와 이어지는 칸만 고른다 (엉뚱한 해를 배제).
    const lim=EXTRAP_MAX_DEG*Math.PI/180;
    let base=null, bk=null;
    for(let r=0;r<=2&&!base;r++){
      for(let dx=-r;dx<=r&&!base;dx++)
        for(let dy=-r;dy<=r&&!base;dy++)
          for(let dz=-r;dz<=r&&!base;dz++){
            if(Math.max(Math.abs(dx),Math.abs(dy),Math.abs(dz))!==r)continue;
            const c=TBL.cells.get(tkey(kx+dx,ky+dy,kz+dz));
            if(!c)continue;
            let jump=0;
            for(let j=0;j<c.length&&j<cur.length;j++)
              jump=Math.max(jump,Math.abs(c[j]-cur[j]));
            if(jump>lim)continue;            // 팔이 튀는 칸 -> 안 쓴다
            base=c;bk=[kx+dx,ky+dy,kz+dz];
          }
    }
    if(!base)return null;
    const out=base.slice();
    const want=[x/g-bk[0], y/g-bk[1], z/g-bk[2]];   // 칸 단위 남은 거리
    for(let ax=0;ax<3;ax++){
      if(Math.abs(want[ax])<1e-9)continue;
      const step=want[ax]>0?1:-1;
      const kk=[bk[0],bk[1],bk[2]]; kk[ax]+=step;
      const nb=TBL.cells.get(tkey(kk[0],kk[1],kk[2]));
      if(!nb)continue;
      // ★ step*step 은 늘 1 이라 방향이 지워졌다 ★
      //   t 는 want*step 이라 이미 부호가 없는 값이다. 거기에 step*step(=1)을
      //   곱하니, 음의 방향(왼쪽·아래·뒤)으로 밀어도 양의 방향으로 더해졌다.
      //   -> "안 민 방향으로 움직이는" 버그의 원인. step 을 한 번만 곱한다.
      const t=Math.max(0,Math.min(1.5,want[ax]*step));     // 과한 외삽 금지
      for(let j=0;j<out.length;j++)out[j]+=(nb[j]-base[j])*t;
    }
    // 기울기를 더한 뒤에도 다시 확인한다 (더하다가 튈 수 있다)
    let jump=0;
    for(let j=0;j<out.length&&j<cur.length;j++)
      jump=Math.max(jump,Math.abs(out[j]-cur[j]));
    if(jump>EXTRAP_MAX_DEG*Math.PI/180)return null;   // 튀면 안 움직인다
    return out;
  }

  /* 조각 밖으로 나가면 미리 새로 받아둔다 */
  function tblCheck(x,y,z){
    if(!TBL.ready||!TBL.center||TBL.loading)return;
    const kx=Math.round(x/TBL.grid),ky=Math.round(y/TBL.grid),kz=Math.round(z/TBL.grid);
    const d=Math.max(Math.abs(kx-TBL.center[0]),Math.abs(ky-TBL.center[1]),
                     Math.abs(kz-TBL.center[2]));
    // 가장자리에서 한참 남았을 때 미리 받는다. 예전엔 8칸(80mm)이라
    // 받는 동안 계속 밀면 경계를 넘어 '표 채우는 중' 이 떴다.
    if(d>TBL.r*0.45)loadChunk(x,y,z);
  }

  /* 전송 루프: 20Hz. 서버 왕복이 밀리지 않게 한 번에 하나만 보낸다 */
  let busy=false, prevT=performance.now();
  setInterval(async()=>{
    const now=performance.now();
    const g=pad();
    if(el('#joyDiag')){
      el('#joyDiag').textContent = !g ? '조이콘 미인식 (아무 버튼이나 누르세요)'
        : (J.on ? '조종 중' : '준비됨 — 스틱을 움직이면 바로 조작');
    }
    // ★ busy 로 건너뛸 때 prevT 를 갱신하면 그동안의 이동량이 통째로
    //   사라진다 -> "뒤로 제끼고 있는데 뚝뚝 끊기며 간다".
    //   실제로 보낼 때만 시계를 넘긴다.
    if(!g||busy)return;
    const dt=Math.min(.2,(now-prevT)/1000); prevT=now;
    const ax=g.axes[0]||0, ay=g.axes[1]||0;
    if(el('#joyP'))el('#joyP').textContent=ay.toFixed(2);
    if(el('#joyR'))el('#joyR').textContent=ax.toFixed(2);
    const SD=J.dead/100;
    /* 데드존을 뺀 뒤 제곱 곡선을 씌운다.
       직선이면 살짝 밀어도 금방 빨라져 미세 조작이 안 된다.
       제곱이면 조금 밀 때 아주 천천히, 끝까지 밀면 최고 속도.
       (사용자: "조금 밀면 천천히, 많이 밀면 빠르게") */
    const st=v=>{const m=Math.abs(v);
      if(m<SD)return 0;
      const t=(m-SD)/(1-SD);
      return Math.sign(v)*t*t;};
    const bs=g.buttons||[];
    const prs=i=>!!(bs[i]&&bs[i].pressed);
    const ux=-st(ay), uy=-st(ax);
    const dx=ux*J.spd*dt, dy=uy*J.spd*dt;
    const dz=((prs(J.bUp)?1:0)-(prs(J.bDn)?1:0))*J.spd*0.6*dt;
    let grip=null;
    // A 버튼: 6축은 그리퍼 열기 (8축 분사는 뺐다, 11-03 도색 제거)
    if(META.joints.length<7){
      if(prs(0)&&J.grip!==true){grip=true;J.grip=true;}
      if(prs(2)&&J.grip!==false){grip=false;J.grip=false;}
    }
    if(!dx&&!dy&&!dz&&grip===null)return;
    if(!J.on){ await autoStart(); if(!J.on)return; }   // 처음 움직일 때 자동 시작
    if(!TBL.ready&&!TBL.loading)loadChunk();           // 첫 진입 시 표 받기

    /* ① 브라우저에 있는 표로 즉시 찾기 (왕복 없음 = 안 끊긴다) */
    if(TBL.ready&&TBL.pos){
      const nx=TBL.pos[0]+dx, ny=TBL.pos[1]+dy, nz=TBL.pos[2]+dz;
      const q=tblInterp(nx,ny,nz)||tblFind(nx,ny,nz);
      if(q){
        TBL.pos=[nx,ny,nz]; TBL.q=q;
        // 뷰어에 직접 전달 — 서버 폴링을 기다리지 않고 즉시 그린다
        const vf=document.querySelector('#viewer');
        if(vf&&vf.contentWindow)
          vf.contentWindow.postMessage({type:'joyq',q:q},'*');
        if(el('#joyPos'))el('#joyPos').textContent=
          TBL.pos.map(v=>(v*1000).toFixed(0)).join(', ')+' mm';
        if(el('#joyBlk'))el('#joyBlk').textContent='—';
        tblCheck(nx,ny,nz);
        // 서버에는 결과만 알린다 (응답을 기다리지 않는다)
        if(!busy){
          busy=true;
          fetch('/api/call',{method:'POST',
            headers:{'Content-Type':'application/json'},
            body:JSON.stringify({fn:'joy_apply','args':[q,nx,ny,nz]})})
            .catch(()=>{}).finally(()=>{busy=false;});
        }
        return;
      }
      /* ★ 표 밖 ★ 서버 IK(40ms)를 기다리면 그 순간부터 뚝뚝 끊긴다.
         표는 이웃끼리 매끄럽게 이어져 있으므로, 가장 가까운 칸의 자세에서
         '방금 움직인 방향' 만큼 이어서 밀어준다 (기울기 외삽).
         이러면 표가 뒤에서 채워지는 동안에도 화면은 계속 흐른다. */
      const ext=tblExtrap(nx,ny,nz,TBL.q);
      if(ext){
        TBL.pos=[nx,ny,nz]; TBL.q=ext;
        if(el('#joyPos'))el('#joyPos').textContent=
          TBL.pos.map(v=>(v*1000).toFixed(0)).join(', ')+' mm';
        if(el('#joyBlk'))el('#joyBlk').textContent='표 채우는 중…';
        const vf=document.querySelector('#viewer');
        if(vf&&vf.contentWindow)
          vf.contentWindow.postMessage({type:'joyq',q:ext},'*');
        tblCheck(nx,ny,nz);
        if(!busy){
          busy=true;
          fetch('/api/call',{method:'POST',
            headers:{'Content-Type':'application/json'},
            body:JSON.stringify({fn:'joy_apply','args':[ext,nx,ny,nz]})})
            .catch(()=>{}).finally(()=>{busy=false;});
        }
        return;
      }
      /* 표에도 없고 이어지는 답도 없으면 ② 서버 IK 로 내려간다.
         (여기서 멈추게 했더니 닭 머리 표를 안 쓰는 로봇(SO-ARM)은
          아예 못 움직였다 — 예전처럼 서버가 풀게 둔다) */
      tblCheck(nx,ny,nz);
    }

    /* ② 표에 없으면 서버가 IK 로 푼다 (예전 방식) */
    if(busy)return;
    busy=true;
    try{
      const res=await fetch('/api/call',{method:'POST',
        headers:{'Content-Type':'application/json'},
        body:JSON.stringify({fn:'joy_move',args:[dx,dy,dz,grip]})});
      const j2=await res.json();
      const o=(j2.outputs&&j2.outputs[0])||{};
      if(o.pos){
        TBL.pos=o.pos.slice(0,3);
        if(el('#joyPos'))el('#joyPos').textContent=
          o.pos.map(v=>(v*1000).toFixed(0)).join(', ')+' mm';
      }
      if(el('#joyBlk'))
        el('#joyBlk').textContent=(o.blocked&&o.blocked.length)?
          o.blocked.join(' ')+' 막힘':'—';
    }catch(e){}
    busy=false;
  },16);        // 60Hz — 로컬 표 + 보간이라 서버를 안 기다린다 (joy_apply 2ms)
})();
