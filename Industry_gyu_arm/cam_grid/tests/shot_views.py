import asyncio, urllib.request
from playwright.async_api import async_playwright
U='http://127.0.0.1:7991'
async def main():
    async with async_playwright() as p:
        b=await p.chromium.launch(args=['--use-gl=swiftshader','--enable-webgl','--ignore-gpu-blocklist'])
        pg=await b.new_page(viewport={'width':960,'height':560}); errs=[]
        pg.on('pageerror',lambda e: errs.append(f'PAGEERROR: {e}'))
        urllib.request.urlopen(U+'/test/pause').read(); urllib.request.urlopen(U+'/seek/40').read()
        await pg.goto(U+'/'); await pg.wait_for_timeout(4000)
        urllib.request.urlopen(U+'/test/fp').read()
        for name in ('both','twin','video'):
            await pg.wait_for_timeout(3000); await pg.screenshot(path=f'/tmp/claude-1000/-home-drboom-2026-project/e9b0ceb6-0f99-4c00-b63f-eb00e4e6256e/scratchpad/view_{name}.png')
            print(name, '->', urllib.request.urlopen(U+'/test/view').read().decode().split('|')[1][:40])
        print('\n'.join(errs) or '오류 없음'); await b.close()
asyncio.run(main())
