import re, pathlib, sys

B=sys.argv[1] if len(sys.argv)>1 else '.'   # where the intermediate SVGs live

FONTS=[('DejaVu Sans Mono Bold','DejaVu Sans Mono, Menlo, Consolas, monospace'),
       ('DejaVu Sans Mono','DejaVu Sans Mono, Menlo, Consolas, monospace'),
       ('DejaVu Sans Bold','DejaVu Sans, Helvetica, Arial, sans-serif'),
       ('DejaVu Sans','DejaVu Sans, Helvetica, Arial, sans-serif')]

def load(p):
    s=pathlib.Path(p).read_text()
    w=float(re.search(r'<svg width="([\d.]+)pt"',s).group(1))
    h=float(re.search(r'height="([\d.]+)pt"',s).group(1))
    vb=re.search(r'viewBox="([^"]+)"',s).group(1)
    inner=s[s.index('>',s.index('<svg')) + 1 : s.rindex('</svg>')]
    inner=re.sub(r'<!--.*?-->','',inner,flags=re.S)
    return w,h,vb,inner

def compose(body,note,out,gap=26,align='left',indent=8):
    bw,bh,bvb,bi=load(body); nw,nh,nvb,ni=load(note)
    W=max(bw,nw+indent); H=bh+gap+nh
    nx = indent if align=='left' else (W-nw)/2
    s=[f'<?xml version="1.0" encoding="UTF-8" standalone="no"?>',
       f'<!-- Hexapod state machine, generated from Hexapod.cpp with graphviz -->',
       f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
       f'width="{W:.0f}pt" height="{H:.0f}pt" viewBox="0 0 {W:.2f} {H:.2f}">',
       f'<rect width="100%" height="100%" fill="white"/>',
       f'<svg x="0" y="0" width="{bw:.2f}" height="{bh:.2f}" viewBox="{bvb}" overflow="visible">{bi}</svg>',
       f'<svg x="{nx:.2f}" y="{bh+gap:.2f}" width="{nw:.2f}" height="{nh:.2f}" viewBox="{nvb}" overflow="visible">{ni}</svg>',
       '</svg>']
    s='\n'.join(s)
    for a,b in FONTS: s=s.replace(f'font-family="{a}"',f'font-family="{b}"')
    pathlib.Path(out).write_text(s); print(out, f'{W:.0f}x{H:.0f}pt')

compose(f'{B}/sm_operator_body.svg',f'{B}/sm_operator_note.svg',
        '../state-machine.svg')
compose(f'{B}/sm_detail_body.svg',f'{B}/sm_detail_note.svg',
        '../state-machine-detail.svg')
