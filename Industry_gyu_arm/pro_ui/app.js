/* ================= ARM STUDIO PRO — frontend ================= */
'use strict';
const $=s=>document.querySelector(s), $$=s=>[...document.querySelectorAll(s)];

let META=null;          // /api/meta
let ST=null;            // 마지막 /api/state
let sel=null;           // 파이프라인 선택 블록 라벨
let selPose='base';     // 자세 탭 선택
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
    await new Promise(r=>setTimeout(r,500));
    let j;try{j=await(await fetch('/api/job')).json();}catch(e){continue;}
    const s=j.busy?(j.status||j.prog):j.status;
    if(s&&s!==last){log(s);last=s;}
    setTicker(j.busy?(j.prog||j.status||'작업 중'):(s||'완료'));
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
  try{$('#mapBox').innerHTML=
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
  /* 상단 전광판: 작업 중이면 진행 상황, 아니면 마지막 로그 */
  if(s.busy||s.running)setTicker(s.prog||lastLog);
  const execLab=Object.keys(META.exec_map).find(k=>META.exec_map[k]===s.exec)||s.exec;
  $('#robotChip').textContent=(s.running?'▶ 동작 중 · ':'')+execLab;
  $('#robotChip').classList.toggle('on',s.running||s.busy);

  /* 자세 목록 */
  if(changed('poses',[s.poses.map(p=>p.name),selPose])){
    const el=$('#poseList');el.innerHTML='';
    s.poses.forEach(p=>{
      const b=document.createElement('button');
      b.textContent=p.name;if(p.name===selPose)b.classList.add('on');
      b.onclick=async()=>{selPose=p.name;HASH.sliders=null;
        await call('select_pose',p.name);};
      el.append(b);
    });
    opts($('#mvPose'),s.poses.map(p=>p.name),$('#mvPose').value||'base');
  }
  /* 슬라이더 값 */
  const pd=s.poses.find(p=>p.name===selPose);
  if(pd&&changed('sliders',[selPose,pd.deg])){
    $$('#sliders input').forEach((r,i)=>{r.value=pd.deg[i];
      r.closest('.sliderRow').querySelector('.jval').textContent=pd.deg[i]+'°';});
  }
  if(changed('ftoff',s.ft_off))$('#ftOff').textContent=s.ft_off||'';

  /* 보정 로봇 / 조인트 */
  if(changed('cal',[s.cal,s.cal_joints,ftJoint])){
    const cur=Object.keys(META.cal_map).find(k=>META.cal_map[k]===s.cal);
    segBtns($('#calRobot'),Object.keys(META.cal_map),cur,
      l=>call('cal_robot',l));
    if(!s.cal_joints.includes(ftJoint))ftJoint=s.cal_joints[0]||'J1';
    segBtns($('#ftJoints'),s.cal_joints,ftJoint,
      l=>{ftJoint=l;HASH.cal=null;render(ST);});
  }

  /* 파이프라인 */
  if(changed('seq',[s.seq.map(x=>x.label),sel])){
    const el=$('#pipeline');el.innerHTML='';
    if(!s.seq.length)el.innerHTML='<div class="hint">블록이 없습니다. 위에서 자세를 추가하세요.</div>';
    s.seq.forEach(it=>{
      const d=document.createElement('div');
      d.className='blk '+(it.kind==='brick'?
        (it.label.includes('놓기')?'brick rel':'brick'):it.kind);
      d.textContent=it.label;
      if(it.label===sel)d.classList.add('on');
      d.onclick=async()=>{sel=(sel===it.label)?null:it.label;
        await refresh();fetchMap();};
      el.append(d);
    });
  }
  /* 선택 블록 패널 */
  const kind=s.seq.find(x=>x.label===sel)?.kind;
  $('#segPanel').classList.toggle('hidden',kind!=='move');
  $('#ignPanel').classList.toggle('hidden',kind!=='pose');
  if(kind==='move'&&s.seg&&changed('seg',s.seg)){
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
  if(kind==='pose'&&changed('ign',[s.obstacles,s.ign_sel])){
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
  /* 간편 이동 설정 */
  if(changed('simple',s.simple)){
    segBtns($('#spPrefer'),['x 먼저','y 먼저'],
      s.simple.prefer==='x'?'x 먼저':'y 먼저',l=>simpleSet(l));
    $('#spClear').value=s.simple.clear_cm;$('#spClearV').value=s.simple.clear_cm;
    $('#spExtra').value=s.simple.extra_cm;$('#spExtraV').value=s.simple.extra_cm;
  }

  /* 환경: 썸네일 카드 (플레이트 제외, 접두어 제거, 드래그=3D 배치) */
  if(changed('env',[s.env,s.env_on,s.names])){
    const el=$('#envList');el.innerHTML='';
    s.env.filter(id=>!id.toLowerCase().includes('plate')).forEach(id=>{
      const nm=shortNm(id,s.names);
      const c=document.createElement('div');c.className='thumbCard';
      c.draggable=true;c.title=id+' — 3D 화면으로 끌어다 놓으면 이동';
      c.innerHTML=`<img src="/api/thumb/${encodeURIComponent(id)}" draggable="false">
        <div class="thumbNm">${esc(nm)}</div>
        <input type="checkbox"${s.env_on.includes(id)?' checked':''} title="표시">`;
      c.ondragstart=e=>e.dataTransfer.setData('text/plain',id);
      c.querySelector('input').onchange=()=>{
        const on=$$('#envList .thumbCard input:checked')
          .map(i=>i.closest('.thumbCard').dataset.id);
        // 플레이트는 목록에 없으므로 항상 켜진 것으로 함께 전송
        call('env',[...on,...s.env.filter(x=>x.toLowerCase().includes('plate')
                                             &&s.env_on.includes(x))]);
      };
      c.dataset.id=id;
      el.append(c);
    });
  }
  if(changed('plates',[s.plates,s.plate_info])){
    opts($('#plateSel'),s.plates,$('#plateSel').value);
    $('#plateInfo').textContent=s.plate_info||'(없음)';
  }
  if(changed('rpos',s.robot_pos))$('#robotPos').textContent=s.robot_pos;

  /* 공간 선택 */
  if(changed('peg',s.peg)){
    $('#pegView').checked=s.peg.on;
    opts($('#pegSel'),s.peg.choices,s.peg.sel||s.peg.choices[0]);
  }
  if(changed('bricks',[s.bricks,s.docking,s.names])){
    opts2($('#brickSel'),s.bricks,$('#brickSel').value,s.names);
    opts2($('#dockSel'),s.docking,$('#dockSel').value,s.names);
    opts2($('#nmTarget'),[...s.bricks,...s.env],$('#nmTarget').value,s.names);
  }
}

/* ---------- 간편 이동 설정 전송 ---------- */
function simpleSet(preferLab){
  const p=preferLab||($('#spPrefer .on')?.textContent)||'x 먼저';
  call('simple_set',p,+$('#spClear').value,+$('#spExtra').value).then(fetchMap);
}

/* ---------- 초기화 ---------- */
async function boot(){
  META=await(await fetch('/api/meta')).json();

  /* 뷰어 */
  const host=location.hostname;
  $('#viewer').src=`http://${host}:${META.viewer_port}/`;   // 3D 뷰 전용 (RViz 제거)

  /* 전광판 클릭 = 전체 로그 오버레이 */
  $('#ticker').onclick=()=>$('#logOverlay').classList.remove('hidden');
  $('#logClose').onclick=()=>$('#logOverlay').classList.add('hidden');
  $('#logOverlay').onclick=e=>{
    if(e.target.id==='logOverlay')$('#logOverlay').classList.add('hidden');};

  /* 탭 */
  $$('#tabs button').forEach(b=>b.onclick=()=>{
    $$('#tabs button').forEach(x=>x.classList.remove('on'));
    $$('.pane').forEach(x=>x.classList.remove('on'));
    b.classList.add('on');$('#tab-'+b.dataset.tab).classList.add('on');
    if(b.dataset.tab==='sim')fetchMap();   // 맵은 ② 시뮬레이션 탭에 있음
  });

  /* 슬라이더 생성 */
  const sl=$('#sliders');
  META.joints.forEach((j,i)=>{
    const d=document.createElement('div');d.className='sliderRow';
    d.innerHTML=`<span class="jname">${j.label}</span>
      <input type="range" min="${j.min}" max="${j.max}" step="0.1" value="0">
      <span class="jval">0°</span>`;
    const r=d.querySelector('input');
    r.oninput=()=>d.querySelector('.jval').textContent=r.value+'°';
    r.onchange=()=>{HASH.sliders=null;
      call('set_sliders',selPose,...$$('#sliders input').map(x=>+x.value));};
    sl.append(d);
  });

  /* 정적 선택지 */
  opts($('#ftStep'),META.steps,'1');
  opts($('#speed'),META.speeds,'1.5x');
  opts($('#macroSlot'),META.macro_slots,META.macro_slots[0]);
  $('#pegH').max=META.peg_h_max;$('#pegHV').value=$('#pegH').value;

  /* ---- 자세 탭 ---- */
  $('#btnGripOpen').onclick=()=>call('pose_grip',selPose,true);
  $('#btnGripClose').onclick=()=>call('pose_grip',selPose,false);
  $('#btnFtGo').onclick=()=>call('ft_goto',selPose);
  $('#btnFtMinus').onclick=()=>call('ft_step',selPose,ftJoint,$('#ftStep').value,-1);
  $('#btnFtPlus').onclick=()=>call('ft_step',selPose,ftJoint,$('#ftStep').value,1);
  $('#btnFtGOpen').onclick=()=>call('ft_grip',selPose,true);
  $('#btnFtGClose').onclick=()=>call('ft_grip',selPose,false);
  $('#btnFtApply').onclick=()=>call('ft_apply',selPose);
  $('#btnFtReset').onclick=()=>call('ft_reset',selPose);
  $('#btnFtSave').onclick=()=>call('ft_save',selPose);
  $('#btnCalOff').onclick=()=>call('cal_torque_off',
    $('#calRobot .on')?.textContent||META.cal_labels[0]);
  $('#btnCalCap').onclick=()=>call('cal_capture',
    $('#calRobot .on')?.textContent||META.cal_labels[0]);
  $('#btnDup').onclick=()=>call('dup',selPose,$('#newName').value);
  $('#btnDel').onclick=()=>{
    if(confirm(`자세 '${selPose}' 를 삭제할까요?`)){
      call('del',selPose);selPose='base';HASH.poses=null;}
  };
  $('#btnSave').onclick=()=>call('save');
  $('#btnLoad').onclick=()=>call('load');
  $('#btnPath').onclick=()=>call('add_path',$('#pathIn').value);
  $('#upDir').onchange=e=>upload(e.target.files,true);
  $('#upStl').onchange=e=>upload(e.target.files,false);

  /* ---- 이동 탭 ---- */
  $('#btnSeqAdd').onclick=async()=>{await call('seq_add',$('#mvPose').value);fetchMap();};
  $('#btnGripAddOpen').onclick=()=>call('seq_grip',true);
  $('#btnGripAddClose').onclick=()=>call('seq_grip',false);
  $('#btnBlkUp').onclick=()=>sel&&call('seq_move',sel,-1).then(()=>{sel=null;fetchMap();});
  $('#btnBlkDn').onclick=()=>sel&&call('seq_move',sel,1).then(()=>{sel=null;fetchMap();});
  $('#btnBlkDel').onclick=()=>sel&&call('seq_del',sel).then(()=>{sel=null;refresh();fetchMap();});
  $('#segMode').onchange=()=>call('seg_mode',sel,$('#segMode').value);
  $('#segAx').onchange=()=>call('seg_ax',sel,$('#segAx').value);
  $('#segRot').onchange=()=>call('seg_rot',sel,$('#segRot').value);
  $('#btnRec').onclick=()=>call('rec_toggle',sel);
  $('#btnIgnSave').onclick=()=>call('ign_save',sel,
    $$('#ignList input:checked').map(i=>i.value)).then(fetchMap);
  $('#btnRStat').onclick=async()=>{
    const r=await call('robot_status');
    if(r?.outputs)$('#robotStat').textContent=r.outputs;
  };
  const runArgs=()=>['간편 이동',META.modes[0],'x-z-y',
    $('#speed').value,$('#loop').checked,false];
  $('#btnPreview').onclick=()=>call('preview',...runArgs());   // ② 시뮬레이션
  $('#btnPreview2').onclick=()=>call('preview',...runArgs());  // ③ 연동
  $('#btnRun').onclick=()=>call('run',...runArgs());
  $('#btnStop2').onclick=()=>call('stop');
  $('#btnStopSim').onclick=()=>call('stop');
  $('#btnToff').onclick=()=>call('torque_off');
  $('#btnEstop').onclick=()=>call('stop');
  $('#btnMRec').onclick=()=>call('macro_rec',$('#macroSlot').value);
  $('#btnMClr').onclick=()=>call('macro_clear',$('#macroSlot').value);
  $('#btnMapUndo').onclick=()=>call('map_undo',sel).then(fetchMap);
  $('#btnMapClear').onclick=()=>call('map_clear',sel).then(fetchMap);
  $('#btnMapRef').onclick=fetchMap;
  $('#btnPathSave').onclick=()=>call('path_save',sel).then(fetchMap);
  $('#btnPathDel').onclick=()=>call('path_del',sel).then(fetchMap);
  $('#spClear').oninput=()=>$('#spClearV').value=$('#spClear').value;
  $('#spExtra').oninput=()=>$('#spExtraV').value=$('#spExtra').value;
  $('#spClear').onchange=()=>simpleSet(null);
  $('#spExtra').onchange=()=>simpleSet(null);
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

  /* ---- 공간 선택 탭 ---- */
  $('#pegView').onchange=e=>call('peg_view',e.target.checked,+$('#pegH').value);
  $('#pegH').oninput=()=>$('#pegHV').value=$('#pegH').value;
  $('#pegH').onchange=()=>{if($('#pegView').checked)call('peg_height',+$('#pegH').value);};
  $('#pegSel').onchange=()=>call('peg_pick',$('#pegSel').value);
  $('#btnPegGo').onclick=()=>call('peg_goto',$('#pegSel').value,+$('#pegH').value);
  $('#btnPegPose').onclick=()=>call('peg_save_pose',
    $('#pegSel').value,+$('#pegH').value,$('#pegPName').value);
  $('#btnPlace').onclick=()=>call('place',$('#brickSel').value,$('#dockSel').value);
  $('#btnGrasp').onclick=()=>call('grasp',$('#brickSel').value);
  $('#btnRelease').onclick=()=>call('release',$('#dockSel').value);
  $('#btnGraspAdd').onclick=()=>call('seq_grasp',$('#brickSel').value);
  $('#btnReleaseAdd').onclick=()=>call('seq_release',$('#dockSel').value);
  $('#btnNm').onclick=()=>call('set_name',
    $('#nmTarget').value,$('#nmText').value,$('#nmColor').value);

  /* 마이크 (음성 인식은 추후 연결 - 지금은 안내만) */
  $('#btnMic').onclick=()=>{
    log('음성 인식은 준비 중입니다. 아래 [블록 추가] 로 직접 만들 수 있어요.');
  };

  /* 첫 로드 + 주기 폴링 */
  await refresh();fetchMap();
  log('ARM STUDIO 준비 완료');
  setInterval(()=>{if(!document.hidden)refresh();},2000);
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
