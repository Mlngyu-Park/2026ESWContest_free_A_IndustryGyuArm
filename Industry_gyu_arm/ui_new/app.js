/* ARM STUDIO — 새 UI (ui_new)
   기존 pro_ui 와 같은 백엔드 API 를 쓴다. 기존 파일은 건드리지 않는다.
   기능은 기존 화면과 1:1 로 맞춘다 (빠진 기능이 있으면 안 됨). */
'use strict';
const $=s=>document.querySelector(s), $$=s=>[...document.querySelectorAll(s)];
let META=null, ST=null, sel=null, selPose='base', nmSel=null, ftJoint='J1';
const H={};
const esc=s=>String(s).replace(/[&<>"]/g,c=>
  ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const changed=(k,v)=>{const j=JSON.stringify(v);
  if(H[k]===j)return false;H[k]=j;return true;};
const short=id=>String(id).replace(/Industry_gyu_arm_\d*_?/,'')
  .replace(/^\d{8}_?/,'').replace(/_-_/g,'').replace(/^[-_ ]+/,'')||id;
const nameOf=(id,s)=>(s.names&&s.names[id]&&s.names[id].name)||short(id);
const colorOf=(id,s)=>s.names&&s.names[id]&&s.names[id].color;

let lastLog='준비 중';
function log(msg,err){
  if(!msg)return;
  const c=$('#console'),d=document.createElement('div');
  const t=new Date().toTimeString().slice(0,8);
  d.innerHTML=`<span class="t">${t}</span>`+(err?'<span class="e"></span>':'');
  (err?d.querySelector('.e'):d).append(msg);
  c.append(d);c.scrollTop=c.scrollHeight;
  while(c.children.length>300)c.firstChild.remove();
  lastLog=String(msg);tick(lastLog);
}
function tick(t){const e=$('#tickerText');if(e.textContent!==t)e.textContent=t;}

/* ---------- API ---------- */
async function call(fn,...args){
  try{
    const r=await fetch('/api/call',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({fn,args})});
    const j=await r.json();
    if(!j.ok){log(`${fn}: ${j.error||'실패'}`,true);return null;}
    if(j.job)await pollJob(); else if(j.status)log(j.status);
    await refresh();
    return j;
  }catch(e){log(`${fn}: ${e}`,true);return null;}
}
async function pollJob(){
  $('#ticker').classList.add('busy');
  let last='';
  for(;;){
    await new Promise(r=>setTimeout(r,250));
    let j;try{j=await(await fetch('/api/job')).json();}catch(e){continue;}
    const s=j.busy?(j.status||j.prog):j.status;
    if(s&&s!==last){log(s);last=s;}
    tick(j.busy?(j.prog||j.status||'작업 중'):(s||'완료'));
    await refresh();
    if(!j.busy){if(j.error)log(j.error,true);break;}
  }
  $('#ticker').classList.remove('busy');
}
async function refresh(){
  try{ST=await(await fetch('/api/state?sel='+encodeURIComponent(sel||'')
    +'&pose='+encodeURIComponent(selPose||''))).json();render(ST);}catch(e){}
}

/* ---------- 블록 라벨 해석 ---------- */
function parts(it){
  const m=String(it.label||'').match(/^([■◆◈➜▼▲])\s*(\d+)\.\s*(.*)$/);
  const body=m?m[3]:it.label, k=m?m[1]:'';
  if(k==='➜')return{c:'move',a:'이동',o:body.replace(/^이동:\s*/,''),kind:'obj'};
  if(k==='◆')return{c:'grip',a:'그리퍼',o:body.includes('열')?'열기 ✋':'닫기 ✊',
                    kind:'grip'};
  if(k==='▼')return{c:'grip',a:'하강',o:'',kind:''};
  if(k==='▲')return{c:'grip',a:'상승',o:'',kind:''};
  if(body.startsWith('잡기'))
    return{c:'grab',a:'잡기',o:body.replace(/^잡기:\s*/,''),kind:'brick'};
  if(body.startsWith('놓기'))
    return{c:'rel',a:'놓기',o:body.replace(/^놓기:\s*/,''),kind:'dock'};
  return{c:'move',a:body,o:'',kind:'pose'};
}
const idxOf=l=>{const m=String(l||'').match(/^[■◆◈➜▼▲]\s*(\d+)\./);
  return m?+m[1]:-1;};

/* ---------- 렌더 ---------- */
function render(s){
  if(!s)return;
  const RA=s.run_at||{};

  /* 상단 상태 */
  const cc=$('#connChip');
  cc.querySelector('span').textContent=s.running?'동작 중':'대기';
  cc.classList.toggle('on',!!s.running);
  const el=(META&&Object.keys(META.exec_map||{})
    .find(k=>META.exec_map[k]===s.exec))||s.exec||'SIM';
  $('#robotChip').textContent=el;
  if(changed('hud',[s.robot_pos,s.ee]))
    $('#hudPos').textContent=(s.robot_pos||'').replace(/\s+/g,' ');

  /* ===== 공정 (블록) ===== */
  if(changed('pipe',[s.seq.map(x=>x.label),sel,RA.blk,RA.sub,s.seek_blk,
                     s.names,s.reach])){
    const el=$('#pipeline');el.innerHTML='';
    const cap=t=>{const d=document.createElement('div');
      d.className='blk cap';d.textContent=t;el.append(d);};
    cap('시작');
    if(!s.seq.length){
      const h=document.createElement('div');h.className='hintline';
      h.textContent='위 블록을 끌어다 놓으세요';el.append(h);
    }
    let n=0;
    s.seq.forEach(it=>{
      if(it.kind==='move')return;
      const bi=idxOf(it.label),p=parts(it);
      const exp=(s.seq_exp||[])[bi]||{};
      const d=document.createElement('div');
      d.className='blk '+p.c;
      if(RA.running&&RA.blk===bi)d.classList.add('run');
      if(!RA.running&&s.seek_blk===bi)d.classList.add('pick');
      if(it.label===sel)d.style.outline='1.5px solid var(--fg-2)';
      d.draggable=true;
      d.innerHTML=`<span class="no">${String(++n).padStart(2,'0')}</span>`
        +`<span>${esc(p.a)}</span>`;
      if(p.o){
        const o=document.createElement('span');
        o.className='obj';o.textContent=p.o;
        const col=it.target&&colorOf(it.target,s);
        if(col){o.style.background=col;o.style.color='#111';}
        if(p.kind==='grip'){o.style.cursor='pointer';
          o.title='클릭 = 열기/닫기 전환';
          o.onclick=async e=>{e.stopPropagation();H.pipe=null;
            await call('seq_set',it.label,p.o.includes('열')?'@close':'@open');};}
        d.append(o);
      }
      if(exp.steps&&exp.steps.length){
        const ss=document.createElement('span');ss.className='steps';
        exp.steps.forEach(t=>{
          const b=document.createElement('span');
          b.className='st'+(RA.running&&RA.blk===bi&&RA.sub===t?' on':'');
          b.textContent=t;ss.append(b);
        });
        d.append(ss);
      }
      const rch=s.reach&&s.reach[it.label];
      if(rch&&rch.ok===false){
        const e2=document.createElement('span');
        e2.className='err';e2.textContent='!';
        const why=rch.why||'현재 로봇 위치에서 도달 불가';
        e2.title=why;
        e2.onclick=ev=>{ev.stopPropagation();log('⚠ '+it.label+' — '+why,true);};
        e2.onmouseenter=()=>tick('⚠ '+why);
        e2.onmouseleave=()=>tick(lastLog);
        d.append(e2);
      }
      d.onclick=()=>{sel=sel===it.label?null:it.label;H.pipe=null;refresh();};
      d.oncontextmenu=e=>{e.preventDefault();menu(e,bi,it,p,s);};
      d.ondragstart=e=>{e.dataTransfer.setData('pipe',it.label);
        e.dataTransfer.setData('text/plain','');
        e.dataTransfer.effectAllowed='move';};
      /* 사물 알약을 블록 위에 떨어뜨리면 대상 교체 */
      if(p.kind&&p.kind!=='grip'){
        d.ondragover=ev=>{const t=[...ev.dataTransfer.types];
          const ok=p.kind==='obj'?(t.includes('k-brick')||t.includes('k-dock'))
                                 :t.includes('k-'+p.kind);
          if(ok){ev.preventDefault();ev.stopPropagation();
            d.style.borderColor='var(--accent)';}};
        d.ondragleave=()=>d.style.borderColor='';
        d.ondrop=async ev=>{
          d.style.borderColor='';
          const a=JSON.parse(ev.dataTransfer.getData('arg')||'{}');
          if(!a.v)return;
          ev.preventDefault();ev.stopPropagation();H.pipe=null;
          const item=p.kind==='obj'?'@goto:'+a.v
            :a.k==='brick'?'@grasp:'+a.v:'@release:'+a.v;
          await call('seq_set',it.label,item);
        };
      }
      el.append(d);
    });
    cap('끝');
  }

  /* ===== 팔레트 ===== */
  if(changed('pal',[s.bricks_placed,s.docking,s.env_on,s.names])){
    const P=$('#blkPalette');P.innerHTML='';
    const dks=(s.docking||[]).filter(d=>(s.env_on||[]).includes(d));
    const brs=s.bricks_placed||[];
    const act=(txt,item)=>{
      const c=document.createElement('div');
      c.className='chip';c.textContent=txt;c.draggable=true;
      c.ondragstart=e=>{e.dataTransfer.setData('blk',item);
        e.dataTransfer.setData('text/plain','');
        e.dataTransfer.effectAllowed='copy';};
      P.append(c);
    };
    const t=dks[0]||brs[0];
    if(t)act('이동 →','@goto:'+t);
    if(brs.length)act('잡기','@grasp:'+brs[0]);
    if(dks.length)act('놓기','@release:'+dks[0]);
    if(brs.length&&dks.length)
      act('옮기기','PAIR:@grasp:'+brs[0]+'|@release:'+dks[0]);
    act('열기 ✋','@open');act('하강 ▼','@down');
    act('상승 ▲','@up');act('닫기 ✊','@close');
    [...brs,...dks].forEach(id=>{
      const c=document.createElement('div');
      c.className='chip obj';c.textContent=nameOf(id,s);c.draggable=true;
      const col=colorOf(id,s);
      if(col)c.style.background=col;
      c.ondragstart=e=>{
        const k=brs.includes(id)?'brick':'dock';
        e.dataTransfer.setData('arg',JSON.stringify({k,v:id}));
        e.dataTransfer.setData('k-'+k,'1');
        e.dataTransfer.setData('text/plain','');
        e.dataTransfer.effectAllowed='copy';};
      P.append(c);
    });
  }

  /* ===== 타임바 ===== */
  const cnt=s.seq.filter(x=>x.kind!=='move').length;
  const rg=$('#tbRange');
  if(+rg.max!==Math.max(0,cnt-1))rg.max=Math.max(0,cnt-1);
  if(changed('ticks',[cnt,RA.blk,RA.running,s.seek_blk])){
    const T=$('#tbTicks');T.innerHTML='';
    for(let i=0;i<cnt;i++){
      const b=document.createElement('span');
      if(RA.running&&i<RA.blk)b.className='done';
      else if(RA.running&&i===RA.blk)b.className='cur';
      else if(!RA.running&&s.seek_blk===i)b.className='cur';
      T.append(b);
    }
  }
  if(RA.running){
    if(RA.blk>=0)rg.value=RA.blk;
    $('#tbNow').textContent=`${RA.blk+1} / ${cnt||1}`+(RA.sub?' · '+RA.sub:'');
    $('#tbTime').textContent=(RA.elapsed||0).toFixed(1)+'초';
  }else if(s.seek_blk>=0){
    rg.value=s.seek_blk;
    $('#tbNow').textContent=`${s.seek_blk+1} / ${cnt}`;
    $('#tbTime').textContent='정지';
  }else{
    $('#tbNow').textContent=cnt?'대기':'블록 없음';
    $('#tbTime').textContent='';
  }
  $('#btnSuggestGo').classList.toggle('hidden',!s.suggest_ready);
  render2(s);                       // 공장/로봇 탭
}

/* ---------- 공장/로봇 탭 렌더 (render 에서 이어서 호출) ---------- */
function render2(s){
  /* 설비 썸네일 */
  if(changed('env',[s.env,s.env_on,s.names,s.bricks,s.bricks_placed])){
    const el=$('#envList');el.innerHTML='';
    const mk=(id,placed)=>{
      const d=document.createElement('div');
      d.className='thumb';d.draggable=true;
      d.title=placed?'끌면 이동 · 3D 우클릭 = 삭제':'끌어서 공장에 배치';
      const col=colorOf(id,s);
      d.innerHTML=(col?`<span class="sw" style="background:${col}"></span>`:'')
        +`<img src="/api/thumb?id=${encodeURIComponent(String(id).split('#')[0])}"
            onerror="this.style.display='none'">`
        +`<div>${esc(nameOf(id,s))}</div>`;
      d.ondragstart=e=>{
        fetch('/api/call',{method:'POST',
          headers:{'Content-Type':'application/json'},
          body:JSON.stringify({fn:'env_drag',args:[id,!!placed]})}).catch(()=>{});
        e.dataTransfer.setData('text/plain','');
        e.dataTransfer.effectAllowed='copy';};
      return d;
    };
    (s.env||[]).forEach(id=>$('#envList').append(mk(id,(s.env_on||[]).includes(id))));
    (s.bricks||[]).forEach(id=>
      $('#envList').append(mk(id,(s.bricks_placed||[]).includes(id))));
  }
  /* 프리셋 */
  if(changed('fp',s.factory_presets))
    $('#fpSel').innerHTML=Object.keys(s.factory_presets||{})
      .map(k=>`<option>${esc(k)}</option>`).join('');
  /* 플레이트 */
  if(changed('plate',s.plates))
    $('#plateSel').innerHTML=(s.plates||[])
      .map(k=>`<option>${esc(k)}</option>`).join('');
  /* 로봇 위치 */
  if(changed('rpos',s.robot_pos))$('#robotPos').textContent=s.robot_pos||'';
  if(changed('rmove',s.rmove_on))$('#robotMove').checked=!!s.rmove_on;
  /* 이름 대상 */
  if(changed('nm',[s.sel_env,s.names])){
    const id=s.sel_env;
    nmSel=id||null;
    $('#nmCur').textContent=id?nameOf(id,s):'3D에서 클릭';
    $('#nmText').disabled=!id;$('#nmColor').disabled=!id;
    if(id){$('#nmText').value=(s.names[id]&&s.names[id].name)||'';
      const c=colorOf(id,s);if(c)$('#nmColor').value=c;}
  }
  /* 자세 목록 */
  if(changed('poses',[s.poses.map(p=>p.name),selPose])){
    const el=$('#poseList');el.innerHTML='';
    (s.poses||[]).forEach(p=>{
      const b=document.createElement('div');
      b.className='pose'+(p.name===selPose?' on':'');
      b.textContent=p.name;
      b.onclick=async()=>{selPose=p.name;H.sl=null;await call('select_pose',p.name);};
      if(p.name!=='base'){
        const x=document.createElement('span');
        x.className='x';x.textContent='×';x.title='이 자세 삭제';
        x.onclick=async e=>{e.stopPropagation();
          if(!confirm(`자세 '${p.name}' 를 삭제할까요?`))return;
          if(selPose===p.name)selPose='base';
          H.poses=null;await call('del',p.name);};
        b.append(x);
      }
      el.append(b);
    });
  }
  /* 관절 슬라이더 */
  const pd=(s.poses||[]).find(p=>p.name===selPose);
  if(pd&&changed('sl',[selPose,pd.deg])){
    const el=$('#sliders');el.innerHTML='';
    (META.joints||[]).forEach((j,i)=>{
      const row=document.createElement('div');row.className='sl';
      row.innerHTML=`<span>${j.label}</span>`;
      const r=document.createElement('input');
      r.type='range';r.min=j.min;r.max=j.max;r.step=.5;r.value=pd.deg[i];
      const v=document.createElement('span');
      v.className='v';v.textContent=pd.deg[i]+'°';
      r.oninput=()=>v.textContent=r.value+'°';
      r.onchange=()=>{H.sl=null;
        call('set_sliders',selPose,...$$('#sliders input').map(x=>+x.value));};
      row.append(r,v);el.append(row);
    });
  }
  if(changed('ftoff',s.ft_off))$('#ftOff').textContent=s.ft_off||'';
  /* 보정 로봇/관절 버튼 */
  if(changed('cal',[s.cal,s.cal_joints,ftJoint])){
    const cur=Object.keys(META.cal_map||{}).find(k=>META.cal_map[k]===s.cal);
    seg($('#calRobot'),Object.keys(META.cal_map||{}),cur,l=>call('cal_robot',l));
    if(!(s.cal_joints||[]).includes(ftJoint))ftJoint=(s.cal_joints||['J1'])[0];
    seg($('#ftJoints'),s.cal_joints||[],ftJoint,l=>{ftJoint=l;H.cal=null;render(ST);});
  }
  /* 매크로 */
  if(changed('mac',s.macros)){
    const el=$('#macroBtns');el.innerHTML='';
    Object.entries(s.macros||{}).forEach(([k,m])=>{
      const b=document.createElement('div');
      b.className='chip';b.style.cursor='pointer';
      b.textContent=`M${k} ${m.name||''}`;
      b.onclick=()=>call('macro_go',k);
      el.append(b);
    });
  }
}
function seg(el,labels,cur,cb){
  if(!el)return;el.innerHTML='';
  labels.forEach(l=>{
    const b=document.createElement('button');
    b.className='btn sm'+(l===cur?' main':'');
    b.textContent=l;b.onclick=()=>cb(l);el.append(b);
  });
}

/* ---------- 우클릭 메뉴 ---------- */
function menu(e,bi,it,p,s){
  document.querySelectorAll('.ctx').forEach(x=>x.remove());
  const m=document.createElement('div');m.className='ctx';
  const add=(t,fn)=>{const b=document.createElement('button');
    b.textContent=t;b.onclick=async ev=>{ev.stopPropagation();m.remove();await fn();};
    m.append(b);};
  add('⏩ 이 자세로 (바로)',()=>call('seq_seek',bi));
  add('▶ 여기까지 재생',()=>call('seq_goto',bi,$('#speed').value||'1.5x'));
  add('🎯 자세로 등록',()=>call('blk_to_pose',it.label));
  add('🗑 삭제',async()=>{sel=null;H.pipe=null;await call('seq_del',it.label);});
  document.body.append(m);
  const r=m.getBoundingClientRect();
  m.style.left=Math.min(e.clientX,innerWidth-r.width-8)+'px';
  m.style.top=Math.min(e.clientY,innerHeight-r.height-8)+'px';
  const close=ev=>{if(!m.contains(ev.target)){m.remove();
    document.removeEventListener('mousedown',close);}};
  setTimeout(()=>document.addEventListener('mousedown',close),0);
}

/* ---------- 파이프라인 드래그/드롭 ---------- */
function wireDrop(){
  const pl=$('#pipeline'), tr=$('#blkTrash');
  pl.addEventListener('dragover',e=>{e.preventDefault();
    e.dataTransfer.dropEffect='copy';});
  pl.addEventListener('drop',async e=>{
    e.preventDefault();
    const kids=[...pl.querySelectorAll('.blk:not(.cap)')];
    let at=kids.length;
    for(let i=0;i<kids.length;i++){
      const r=kids[i].getBoundingClientRect();
      if(e.clientY<r.top+r.height/2){at=i;break;}
    }
    H.pipe=null;
    const blk=e.dataTransfer.getData('blk');
    if(blk){
      if(blk.startsWith('PAIR:')){            // 옮기기 = 이동+잡기+이동+놓기
        const [a1,a2]=blk.slice(5).split('|');
        await call('seq_insert','@goto:'+a1.split(':')[1],at);
        await call('seq_insert',a1,at+1);
        await call('seq_insert','@goto:'+a2.split(':')[1],at+2);
        await call('seq_insert',a2,at+3);
      }else await call('seq_insert',blk,at);
      return;
    }
    const pipe=e.dataTransfer.getData('pipe');
    if(pipe){sel=null;await call('seq_move_to',pipe,at);return;}
    const a=JSON.parse(e.dataTransfer.getData('arg')||'{}');
    if(a.v)await call('seq_insert',
      a.k==='brick'?'@grasp:'+a.v:'@release:'+a.v,at);
  });
  tr.addEventListener('dragover',e=>{
    if(![...e.dataTransfer.types].includes('pipe'))return;
    e.preventDefault();tr.classList.add('hot');});
  tr.addEventListener('dragleave',()=>tr.classList.remove('hot'));
  tr.addEventListener('drop',async e=>{
    e.preventDefault();tr.classList.remove('hot');
    const lab=e.dataTransfer.getData('pipe');
    if(lab){sel=null;H.pipe=null;await call('seq_del',lab);}
  });
}

/* ---------- 시작 ---------- */
(async function(){
  try{META=await(await fetch('/api/meta')).json();}catch(e){}
  $('#viewer').src=(META&&META.viewer_url)||'http://localhost:7870';
  const opt=(el,list,cur)=>{if(el)el.innerHTML=(list||[])
    .map(v=>`<option${v===cur?' selected':''}>${esc(v)}</option>`).join('');};
  opt($('#speed'),(META&&META.speeds)||['1.0x','1.5x','2.0x'],'1.5x');
  opt($('#ftStep'),(META&&META.ft_steps)||['0.5','1','2','5']);
  opt($('#macroSlot'),(META&&META.macro_slots)||['1','2','3']);
  opt($('#execSel'),Object.keys((META&&META.exec_map)||{}));

  /* 사이드바 탭 */
  const T={sim:'공정',factory:'공장',robot:'로봇'};
  $$('#side .nav[data-tab]').forEach(b=>b.onclick=()=>{
    $$('#side .nav').forEach(x=>x.classList.remove('on'));
    b.classList.add('on');
    $$('.pane').forEach(p=>p.classList.remove('on'));
    $('#pane-'+b.dataset.tab).classList.add('on');
    $('#title').textContent=T[b.dataset.tab];
  });
  /* 기록 */
  $('#navLog').onclick=()=>$('#logWrap').classList.add('on');
  $('#ticker').onclick=()=>$('#logWrap').classList.add('on');
  $('#logClose').onclick=()=>$('#logWrap').classList.remove('on');

  const runArgs=()=>['간편 이동',(META&&META.modes&&META.modes[0])||'','x-z-y',
    $('#speed').value,$('#loop').checked,false];
  const B=(id,fn)=>{const e=$(id);if(e)e.onclick=fn;};

  /* 공정 */
  B('#btnEstop',()=>call('stop'));
  B('#btnPreview',()=>{
    const sb=ST&&ST.seek_blk;
    if(sb>=0)return call('preview_from',sb,$('#speed').value);
    return call('preview',...runArgs());});
  B('#btnRun',()=>{if(confirm('실물 로봇이 움직입니다. 진행할까요?'))
    call('run',...runArgs());});
  B('#btnBlkUp',()=>sel&&call('seq_move',sel,-1));
  B('#btnBlkDn',()=>sel&&call('seq_move',sel,1));
  B('#btnBlkClear',()=>{if(confirm('블록을 전부 지울까요? (Ctrl+Z 로 복구)'))
    {sel=null;H.pipe=null;call('seq_clear');}});
  B('#btnSuggest',()=>call('robot_suggest'));
  B('#btnSuggestGo',()=>call('robot_apply_suggest'));
  const send=async()=>{const t=$('#sayTxt').value.trim();if(!t)return;
    log('🗣 '+t);$('#sayTxt').value='';
    const j=await call('say',t);
    const r=(j&&j.outputs&&j.outputs[0])||{};
    if(r.speed)$('#speed').value=r.speed;
    if(r.run)return call('run',...runArgs());
    if(r.preview)$('#btnPreview').click();};
  B('#btnSay',send);
  $('#sayTxt').addEventListener('keydown',e=>{if(e.key==='Enter')send();});
  $('#tbRange').onchange=()=>call('seq_seek',+$('#tbRange').value);

  /* 마이크 (Spark Whisper 우선, 없으면 브라우저) */
  let sttOK=false;
  fetch('/api/stt_ping').then(r=>r.json()).then(j=>{sttOK=!!j.ok;
    if(sttOK)$('#btnMic').title='Spark Whisper';}).catch(()=>{});
  let rec=null,W=null,on=false;
  $('#btnMic').onclick=async()=>{
    if(sttOK){
      if(on){await wStop();return;}
      try{
        const st=await navigator.mediaDevices.getUserMedia({audio:true});
        const ac=new AudioContext({sampleRate:16000});
        const src=ac.createMediaStreamSource(st);
        const pr=ac.createScriptProcessor(4096,1,1);const ch=[];
        pr.onaudioprocess=e=>ch.push(new Float32Array(e.inputBuffer.getChannelData(0)));
        src.connect(pr);pr.connect(ac.destination);
        W={st,ac,pr,ch};on=true;$('#btnMic').textContent='🔴';
      }catch(e){log('마이크 사용 불가: '+e,true);}
      return;
    }
    const SR=window.SpeechRecognition||window.webkitSpeechRecognition;
    if(!SR){log('음성 인식 불가 — Spark 연결 또는 크롬 필요',true);return;}
    if(on){rec.stop();return;}
    rec=new SR();rec.lang='ko-KR';rec.interimResults=true;let fin='';
    rec.onstart=()=>{on=true;$('#btnMic').textContent='🔴';};
    rec.onresult=e=>{let s='';for(const r of e.results){s+=r[0].transcript;
      if(r.isFinal)fin=s;}$('#sayTxt').value=s;tick('🎤 '+s);};
    rec.onend=()=>{on=false;$('#btnMic').textContent='🎤';
      const s=(fin||$('#sayTxt').value||'').trim();if(s)send();};
    rec.start();
  };
  async function wStop(){
    const {st,ac,pr,ch}=W;W=null;on=false;$('#btnMic').textContent='⏳';
    pr.disconnect();st.getTracks().forEach(t=>t.stop());ac.close();
    const n=ch.reduce((a,c)=>a+c.length,0);const pcm=new Int16Array(n);let o=0;
    for(const c of ch)for(let i=0;i<c.length;i++)
      pcm[o++]=Math.max(-32768,Math.min(32767,c[i]*32767|0));
    const h=new ArrayBuffer(44),v=new DataView(h);
    const ws=(p,s)=>{for(let i=0;i<s.length;i++)v.setUint8(p+i,s.charCodeAt(i));};
    ws(0,'RIFF');v.setUint32(4,36+pcm.byteLength,true);ws(8,'WAVEfmt ');
    v.setUint32(16,16,true);v.setUint16(20,1,true);v.setUint16(22,1,true);
    v.setUint32(24,16000,true);v.setUint32(28,32000,true);
    v.setUint16(32,2,true);v.setUint16(34,16,true);
    ws(36,'data');v.setUint32(40,pcm.byteLength,true);
    try{
      const r=await(await fetch('/api/stt',{method:'POST',
        headers:{'Content-Type':'audio/wav'},body:new Blob([h,pcm.buffer])})).json();
      $('#btnMic').textContent='🎤';
      if(r.ok&&r.text){$('#sayTxt').value=r.text;send();}
      else log('음성 인식 실패: '+(r.error||'안 들렸습니다'),true);
    }catch(e){$('#btnMic').textContent='🎤';log('음성 오류: '+e,true);}
  }

  /* 공장 */
  B('#btnFpSave',()=>call('factory_save',$('#fpName').value));
  B('#btnFpLoad',()=>call('factory_load',$('#fpSel').value));
  B('#btnFpDel',()=>call('factory_del',$('#fpSel').value));
  B('#btnFpReset',()=>{if(confirm('새 공장으로 초기화할까요?'))
    call('factory_reset');});
  B('#btnPlateDel',()=>call('plate_remove',$('#plateSel').value));
  B('#btnRobotHome',()=>call('robot_home'));
  $('#robotMove').onchange=()=>call('robot_move_mode',$('#robotMove').checked);
  $('#nmText').oninput=()=>nmSel&&call('set_name',nmSel,$('#nmText').value,
    $('#nmColor').value);
  $('#nmColor').onchange=()=>nmSel&&call('set_name',nmSel,$('#nmText').value,
    $('#nmColor').value);
  $('#upStl').onchange=async e=>{
    const fd=new FormData();
    [...e.target.files].forEach(f=>fd.append('files',f));
    try{const r=await(await fetch('/api/upload',{method:'POST',body:fd})).json();
      log(r.status||'업로드 완료');await refresh();}catch(x){log(''+x,true);}
  };

  /* 로봇 */
  B('#btnRStat',async()=>{const r=await call('robot_status');
    if(r&&r.outputs)$('#robotStat').textContent=r.outputs;});
  $('#execSel').onchange=()=>call('exec_robot',$('#execSel').value);
  B('#btnFtGo',()=>call('ft_goto',selPose));
  B('#btnFtPlus',()=>call('ft_step',selPose,ftJoint,$('#ftStep').value,1));
  B('#btnFtMinus',()=>call('ft_step',selPose,ftJoint,$('#ftStep').value,-1));
  B('#btnFtReset',()=>call('ft_reset',selPose));
  B('#btnFtSave',()=>call('ft_save',selPose));
  B('#btnCalOff',()=>call('cal_torque_off',
    $('#calRobot .main')?$('#calRobot .main').textContent:(META.cal_labels||[''])[0]));
  B('#btnCalCap',()=>call('cal_capture',
    $('#calRobot .main')?$('#calRobot .main').textContent:(META.cal_labels||[''])[0]));
  B('#btnMRec',()=>call('macro_rec',$('#macroSlot').value));
  B('#btnMClr',()=>call('macro_clear',$('#macroSlot').value));
  B('#btnGripOpen',()=>call('ft_grip',selPose,true));
  B('#btnGripClose',()=>call('ft_grip',selPose,false));
  B('#btnToff',()=>call('torque_off'));

  addEventListener('keydown',e=>{
    if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'
      &&!/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)){
      e.preventDefault();H.pipe=null;H.env=null;call('undo');}
    if(e.key==='Escape')$('#logWrap').classList.remove('on');
  });

  wireDrop();
  await refresh();
  log('ARM STUDIO 준비 완료');
  setInterval(()=>{if(!document.hidden)refresh();},1500);
})();
