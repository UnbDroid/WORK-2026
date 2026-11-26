#!/usr/bin/env python3
"""
GRAVA WAYPOINTS COM O ROBÔ REAL.

Leve o robô com o teleop até a posição desejada, PARE, e aperte Enter aqui:
o script lê onde o robô está no mapa (TF map -> base_link), faz a média de
1 segundo de leituras e salva [x, y, yaw] no formato do cenario_X.yaml.

Precisa estar rodando: a localização (slam_toolbox em localização ou AMCL)
e o teleop. O Nav2 não é necessário.

Uso:
  python3 gravar_waypoints.py --saida cenario_novo.yaml --nomes start w1 w2 w3 finish
  python3 gravar_waypoints.py --entrada cenario_2.yaml --saida cenario_novo.yaml

No terminal:
  Enter          = grava a posição atual no waypoint indicado e vai para o próximo
  <nome> + Enter = grava a posição atual com esse nome (ex.: w5)
  v              = volta para o waypoint anterior
  p              = pula (mantém o valor que já existia)
  q              = salva e sai
"""
import argparse
import math
import threading
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class Gravador(Node):
    def __init__(self):
        super().__init__('gravar_waypoints')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def pose_atual(self):
        t = self.buf.lookup_transform('map', 'base_link', Time())
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def pose_media(self, segundos=1.0, n=10):
        xs, ys, ss, cs = [], [], [], []
        for _ in range(n):
            x, y, yaw = self.pose_atual()
            xs.append(x); ys.append(y); ss.append(math.sin(yaw)); cs.append(math.cos(yaw))
            time.sleep(segundos / n)
        return sum(xs) / n, sum(ys) / n, math.atan2(sum(ss), sum(cs))


def carregar(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    p = d.get('/**', d).get('ros__parameters', d)
    nomes = list(p.get('waypoints', []))
    poses = {n: tuple(float(v) for v in p[f'{n}_coords'])
             for n in nomes if isinstance(p.get(f'{n}_coords'), list) and len(p[f'{n}_coords']) == 3}
    return nomes, poses


def salvar(path, nomes, poses):
    linhas = ['/**:', '  ros__parameters:',
              '    waypoints: [' + ', '.join(f'"{n}"' for n in nomes) + ']', '',
              '    # [X, Y, Yaw_em_radianos]  (gravado com o robô pelo gravar_waypoints.py)']
    for n in nomes:
        if n in poses:
            x, y, yaw = poses[n]
            linhas.append(f'    {n}_coords: [{x:.3f}, {y:.3f}, {yaw:.3f}]')
        else:
            linhas.append(f'    # {n}_coords: FALTANDO')
    with open(path, 'w') as f:
        f.write('\n'.join(linhas) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--entrada', help='cenario_X.yaml existente (opcional)')
    ap.add_argument('--saida', required=True)
    ap.add_argument('--nomes', nargs='*')
    a = ap.parse_args()

    nomes, poses = ([], {})
    if a.entrada:
        nomes, poses = carregar(a.entrada)
    if a.nomes:
        nomes = a.nomes
    if not nomes:
        ap.error('informe --entrada ou --nomes')

    rclpy.init()
    node = Gravador()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print('Aguardando a TF map -> base_link (a localização precisa estar rodando)...')
    while True:
        try:
            x, y, yaw = node.pose_atual()
            print(f'OK. Robô agora em [{x:.2f}, {y:.2f}, {yaw:.3f}]')
            break
        except Exception:
            time.sleep(0.5)

    i = 0
    try:
        while 0 <= i < len(nomes):
            n = nomes[i]
            atual = poses.get(n)
            txt = f'[{atual[0]:.2f}, {atual[1]:.2f}, {atual[2]:.3f}]' if atual else 'sem valor'
            cmd = input(f'\n==> {n} ({i + 1}/{len(nomes)}) valor atual: {txt}\n'
                        f'    Posicione o robô e PARE. Enter=gravar | <nome>=gravar com outro nome | '
                        f'v=voltar | p=pular | q=sair: ').strip()
            if cmd.lower() == 'q':
                break
            if cmd.lower() == 'v':
                i = max(0, i - 1)
                continue
            if cmd.lower() == 'p':
                i += 1
                continue
            alvo = n
            if cmd:
                alvo = cmd
                if alvo not in nomes:
                    nomes.insert(i, alvo)
            try:
                pose = node.pose_media()
            except Exception as e:
                print(f'    Não consegui ler a posição: {e}')
                continue
            poses[alvo] = pose
            salvar(a.saida, nomes, poses)
            print(f'    {alvo} gravado: [{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}]  -> {a.saida}')
            i = nomes.index(alvo) + 1
    except (KeyboardInterrupt, EOFError):
        pass
    salvar(a.saida, nomes, poses)
    print(f'\nArquivo salvo: {a.saida}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
