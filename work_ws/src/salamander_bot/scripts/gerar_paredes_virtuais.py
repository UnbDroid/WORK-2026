#!/usr/bin/env python3
"""
Gera a máscara de PAREDES VIRTUAIS (fitas) para o Nav2, alinhada com o mapa da arena.

Você informa cada fita como um segmento em coordenadas do MAPA (metros, as mesmas do RViz
e do cenario_2.yaml) e o script desenha linhas pretas numa imagem do MESMO tamanho,
resolução e origem do mapa. O resultado (virtual_walls.pgm + virtual_walls.yaml) é
carregado por um map_server separado e entra nos costmaps como uma segunda camada estática.

Uso:
  python3 gerar_paredes_virtuais.py --map maps/map_BTT.yaml --out maps/virtual_walls \
      --fita 1.20 0.50 1.20 2.00 \
      --fita 0.00 3.00 2.50 3.00

  --fita x1 y1 x2 y2   (pode repetir; cada uma é uma fita reta)
  --largura 0.10       (largura desenhada em metros; a fita real tem 5 cm, use um pouco mais)

Também aceita um arquivo com uma fita por linha ("x1 y1 x2 y2"): --arquivo fitas.txt

Sem nenhuma --fita, gera uma máscara VAZIA (para provas sem fita):
  python3 gerar_paredes_virtuais.py --map maps/map_BTT.yaml --out maps/virtual_walls_vazia
"""
import argparse
import os
import sys

import yaml
from PIL import Image, ImageDraw


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True, help='YAML do mapa (ex.: maps/map_BTT.yaml)')
    ap.add_argument('--out', required=True, help='Saída sem extensão (ex.: maps/virtual_walls)')
    ap.add_argument('--fita', nargs=4, type=float, action='append', default=[],
                    metavar=('X1', 'Y1', 'X2', 'Y2'), help='Segmento da fita em metros (frame map)')
    ap.add_argument('--arquivo', help='Arquivo texto com "x1 y1 x2 y2" por linha')
    ap.add_argument('--largura', type=float, default=0.10, help='Largura desenhada (m). Padrão 0.10')
    a = ap.parse_args()

    fitas = list(a.fita)
    if a.arquivo:
        with open(a.arquivo) as f:
            for n, line in enumerate(f, 1):
                line = line.split('#')[0].strip()
                if not line:
                    continue
                v = [float(t) for t in line.replace(',', ' ').split()]
                if len(v) != 4:
                    sys.exit(f'Linha {n} de {a.arquivo}: esperado "x1 y1 x2 y2"')
                fitas.append(v)
    if not fitas:
        print('Nenhuma fita informada: gerando máscara VAZIA (para provas sem fita).')

    with open(a.map) as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(a.map)), img_path)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    if len(meta['origin']) > 2 and abs(float(meta['origin'][2])) > 1e-6:
        sys.exit('Mapa com origem rotacionada não é suportado por este script.')

    w, h = Image.open(img_path).size

    # Fundo BRANCO = livre (não altera nada no costmap, pois a camada usa o máximo).
    mask = Image.new('L', (w, h), 254)
    draw = ImageDraw.Draw(mask)
    largura_px = max(1, int(round(a.largura / res)))

    def to_px(x, y):
        return ((x - ox) / res, h - 1 - (y - oy) / res)

    for x1, y1, x2, y2 in fitas:
        p1, p2 = to_px(x1, y1), to_px(x2, y2)
        for p in (p1, p2):
            if not (0 <= p[0] < w and 0 <= p[1] < h):
                print(f'AVISO: ponto ({x1:.2f},{y1:.2f})-({x2:.2f},{y2:.2f}) fora do mapa!')
        draw.line([p1, p2], fill=0, width=largura_px)
        r = largura_px / 2
        for px, py in (p1, p2):   # pontas arredondadas
            draw.ellipse([px - r, py - r, px + r, py + r], fill=0)
        print(f'Fita: ({x1:.2f}, {y1:.2f}) -> ({x2:.2f}, {y2:.2f})')

    out_pgm = a.out + '.pgm'
    out_yaml = a.out + '.yaml'
    mask.save(out_pgm)
    out_meta = {
        'image': os.path.basename(out_pgm),
        'mode': 'trinary',
        'resolution': res,
        'origin': [ox, oy, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with open(out_yaml, 'w') as f:
        yaml.safe_dump(out_meta, f, sort_keys=False)
    print(f'\nGerado: {out_pgm} e {out_yaml} ({len(fitas)} fitas, {largura_px} px de largura)')


if __name__ == '__main__':
    main()
