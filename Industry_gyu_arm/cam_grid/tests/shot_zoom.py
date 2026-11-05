import asyncio, urllib.request
from playwright.async_api import async_playwright
U='http://127.0.0.1:7991'
async def main():
    async with async_playwright() as p:
        b=await p.chromium.launch(args=['--use-gl=swiftshader','--enable-webgl','--ignore-gpu-blocklist'])
        pg=await b.new_page(viewport={'width':800,'height':480}); errs=[]
        pg.on('pageerror',lambda e: errs.append(f'PAGEERROR: {e}'))
        urllib.request.urlopen(U+'/test/pause').read(); urllib.request.urlopen(U+'/seek/40').read()
        await pg.goto(U+'/'); await pg.wait_for_timeout(4000)
        urllib.request.urlopen(U+'/test/fp').read(); urllib.request.urlopen(U+'/test/view').read()   # 트윈만
        await pg.wait_for_timeout(2500); await pg.screenshot(path='/tmp/claude-1000/-home-drboom-2026-project/e9b0ceb6-0f99-4c00-b63f-eb00e4e6256e/scratchpad/z_twin_1.png')
        await pg.mouse.move(400,240)
        for _ in range(8): await pg.mouse.wheel(0,-120); await pg.wait_for_timeout(60)
        await pg.wait_for_timeout(800); await pg.screenshot(path='/tmp/claude-1000/-home-drboom-2026-project/e9b0ceb6-0f99-4c00-b63f-eb00e4e6256e/scratchpad/z_twin_in.png')
        z=await pg.evaluate('typeof camSync!=="undefined"?camSync.zoom:null'); print('트윈만 확대 배율:', z)
        urllib.request.urlopen(U+'/test/view').read(); urllib.request.urlopen(U+'/test/view').read()  # 영상+트윈
        await pg.wait_for_timeout(2500); await pg.screenshot(path='/tmp/claude-1000/-home-drboom-2026-project/e9b0ceb6-0f99-4c00-b63f-eb00e4e6256e/scratchpad/z_both.png')
        for _ in range(8): await pg.mouse.wheel(0,-120); await pg.wait_for_timeout(60)
        await pg.wait_for_timeout(800); await pg.screenshot(path='/tmp/claude-1000/-home-drboom-2026-project/e9b0ceb6-0f99-4c00-b63f-eb00e4e6256e/scratchpad/z_both_after.png')
        print('\n'.join(errs) or '오류 없음'); await b.close()
asyncio.run(main())
