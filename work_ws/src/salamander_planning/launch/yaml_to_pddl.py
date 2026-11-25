"""Traduz um arquivo de equipe (.yaml) do @Work Commander para um problem .pddl
do `salamander-domain` (types: workspace, cube, tag, slot, container)."""
import argparse
import itertools
import os
import sys

import yaml

# NOMES

LOC_START = 'start'
LOC_FINISH = 'finish'

# Waypoints da arena: w1..w9 mesas comuns (WS_n), w10..w13 prateleiras (SH_1..SH_4),
# w14..w15 mesas de precisao (PP, PP_2). Cada prateleira tem UM waypoint (BOTTOM e TOP
# compartilham o workspace).
N_WS = 9
N_SH = 4
SH_BASE = 9           # SH_n -> w{SH_BASE + n}
PP_BASE = 13          # PP -> w14, PP_2 -> w15
SLOTS_ROBO = ['slot1', 'slot2', 'slot3']
COR_PT = {'BLUE': 'azul', 'RED': 'vermelho'}
COR_ID_VISAO = {'BLUE': '-2', 'RED': '-1'}

# O exemplo de problema comeca com (vision-ready) e sem (arm-folded).
# Se o robo realmente comeca dobrado, ponha False.
INICIO_VISION_READY = True


def nome_cubo(tag):
    return f'cube{tag}'


def nome_tag_cubo(tag):       # tag que identifica o cubo (tag-cube)
    return f'tag{tag}'


def nome_tag_mesa(tag):       # tag de referencia da zona de empilhamento (tag-desk)
    return f'ref{tag}'


def nome_ws(area):            # 'WS_4' -> 'w4'
    n = int(area.split('_')[1])
    if not 1 <= n <= N_WS:
        raise ErroContrato(f'{area}: mesa comum so vai de WS_1 a WS_{N_WS}')
    return f'w{n}'


def nome_shelf(area, nivel):  # 'SH_2' -> 'w11' (nivel nao muda o workspace)
    n = int(area.split('_')[1])
    if not 1 <= n <= N_SH:
        raise ErroContrato(f'{area}: prateleira so vai de SH_1 a SH_{N_SH}')
    return f'w{SH_BASE + n}'


def nome_pp(area):            # 'PP' -> 'w14', 'PP_2' -> 'w15'
    n = 1 if area == 'PP' else int(area.split('_')[1])
    return f'w{PP_BASE + n}'


def nome_container(cor, loc):
    return f'container_{COR_PT[cor]}_{loc}{COR_ID_VISAO[cor]}'


# CONFERENCIA DO ARQUIVO (invariantes do contrato)

class ErroContrato(Exception):
    pass


def tipo_area(area):
    if area.startswith('WS_'):
        return 'WS'
    if area.startswith('SH_'):
        return 'SH'
    if area == 'PP' or area.startswith('PP_'):
        return 'PP'
    raise ErroContrato(f'Area desconhecida: {area}')


def posicoes_da_area(area, dados, stacking, nome_estado):
    """Devolve lista de (obj, colocacao). colocacao = (tipo, area, detalhe)."""
    t = tipo_area(area)
    saida = []
    if t == 'WS':
        for i, o in enumerate(dados.get('table') or []):
            saida.append((o, ('table', area, i)))
        for cor, objs in (dados.get('containers') or {}).items():
            for o in objs or []:
                saida.append((o, ('container', area, cor)))
        if 'stack' in dados:
            if area not in stacking:
                raise ErroContrato(f'{nome_estado}.{area}: `stack` numa WS fora de stacking_areas')
            for i, o in enumerate(dados.get('stack') or []):
                saida.append((o, ('stack', area, i)))
    elif t == 'SH':
        for nivel, objs in (dados.get('levels') or {}).items():
            if nivel not in ('BOTTOM', 'TOP'):
                raise ErroContrato(f'{nome_estado}.{area}: nivel invalido {nivel}')
            for i, o in enumerate(objs or []):
                saida.append((o, ('shelf_' + nivel, area, i)))
    elif t == 'PP':
        slots = dados.get('slots')
        if slots is None:
            # arranjo escondido (AMT): sabemos so que esta na PP
            for o in dados.get('obj_ids') or []:
                saida.append((o, ('pp_desconhecido', area, None)))
        else:
            for tag, o in slots.items():
                if o is not None:
                    saida.append((o, ('slot', area, int(tag))))
    return saida


def conferir(doc):
    erros = []
    objetos = {o['id']: o['color'] for o in doc.get('objects') or []}
    if len(objetos) != len(doc.get('objects') or []):
        erros.append('IDs repetidos em `objects`')
    ativas = list(doc.get('active_service_areas') or [])
    stacking = doc.get('stacking_areas') or {}
    pts = doc.get('precision_tables') or {}

    for nome_estado in ('start_state', 'finish_state'):
        estado = doc.get(nome_estado) or {}
        if set(estado) != set(ativas):
            erros.append(f'{nome_estado}: areas {sorted(estado)} != ativas {sorted(ativas)}')
        vistos = []
        for area, dados in estado.items():
            obj_ids = list(dados.get('obj_ids') or [])
            if obj_ids != sorted(obj_ids):
                erros.append(f'{nome_estado}.{area}: obj_ids fora de ordem crescente')
            try:
                pos = posicoes_da_area(area, dados, stacking, nome_estado)
            except ErroContrato as e:
                erros.append(str(e))
                continue
            ids_pos = sorted(o for o, _ in pos)
            if ids_pos != sorted(obj_ids):
                erros.append(f'{nome_estado}.{area}: posicoes {ids_pos} != obj_ids {obj_ids}')
            vistos += obj_ids
            for cor in (dados.get('containers') or {}):
                if {'color': cor, 'at': area} not in (doc.get('containers') or []):
                    erros.append(f'{nome_estado}.{area}: container {cor} nao declarado em `containers`')
                for o in dados['containers'][cor] or []:
                    if objetos.get(o) != cor:
                        erros.append(f'{nome_estado}.{area}: cubo {o} ({objetos.get(o)}) no container {cor}')
            if tipo_area(area) == 'PP' and dados.get('slots') is not None:
                esperado = [int(t) for t in (pts.get(area) or {}).get('slot_tags', [])]
                if [int(t) for t in dados['slots']] != esperado:
                    erros.append(f'{nome_estado}.{area}: chaves de slots != precision_tables.{area}.slot_tags')
            if area in stacking and nome_estado == 'start_state' and obj_ids:
                erros.append(f'start_state.{area}: zona de empilhamento deve comecar vazia')
        if sorted(vistos) != sorted(objetos):
            erros.append(f'{nome_estado}: objetos {sorted(vistos)} != `objects` {sorted(objetos)}')
    if erros:
        raise ErroContrato('Arquivo fora do contrato:\n  - ' + '\n  - '.join(erros))


# TRADUCAO

def traduzir(doc, com_finish=True):
    conferir(doc)
    avisos = []
    task = doc['task_id']
    objetos = {o['id']: o['color'] for o in doc['objects']}
    ativas = list(doc['active_service_areas'])
    stacking = doc.get('stacking_areas') or {}
    pts = doc.get('precision_tables') or {}

    # workspaces
    mapa_areas = {}           # area YAML -> lista de workspaces PDDL
    workspaces = [LOC_START]
    prateleiras = []          # workspaces que recebem (shelf w)
    for area in ativas:
        t = tipo_area(area)
        if t == 'WS':
            locs = [nome_ws(area)]
        elif t == 'SH':
            locs = list(dict.fromkeys([nome_shelf(area, 'BOTTOM'), nome_shelf(area, 'TOP')]))
            prateleiras += locs
        else:
            locs = [nome_pp(area)]
        mapa_areas[area] = locs
        workspaces += locs
    workspaces.append(LOC_FINISH)

    def loc_de(col):
        tipo, area, _ = col
        if tipo == 'shelf_BOTTOM':
            return nome_shelf(area, 'BOTTOM')
        if tipo == 'shelf_TOP':
            return nome_shelf(area, 'TOP')
        if tipo in ('slot', 'pp_desconhecido'):
            return nome_pp(area)
        return nome_ws(area)

    # colocacoes
    inicio, fim = {}, {}
    for nome_estado, destino in (('start_state', inicio), ('finish_state', fim)):
        for area, dados in doc[nome_estado].items():
            for o, col in posicoes_da_area(area, dados, stacking, nome_estado):
                destino[o] = col

    def parado(o):
        a, b = inicio[o], fim[o]
        if a[0] == 'pp_desconhecido':
            return False
        if a[0] in ('table', 'shelf_BOTTOM', 'shelf_TOP') and a[:2] == b[:2]:
            return True       # mesma mesa/nivel: nao precisa ser movido
        return a == b

    # mesa de precisao: so liga a visao (goal `precision-started`). Cubos que comecam E
    # terminam na PP sao resolvidos pela rotina de precisao e ficam fora do PDDL.
    def em_pp(col):
        return col[0] in ('slot', 'pp_desconhecido')

    internos = sorted(o for o in objetos if em_pp(inicio[o]) and em_pp(fim[o]))
    ativos = sorted(o for o in objetos if o not in internos)
    pp_ativas = [a for a in ativas if tipo_area(a) == 'PP']

    # pilhas
    for area in stacking:
        pilha = (doc['finish_state'].get(area) or {}).get('stack') or []
        if len(pilha) > 2:
            raise ErroContrato(
                f'{area}: pilha com {len(pilha)} cubos. stack_cube_base/stack_cube_top do domain '
                'so montam pilha de 2; precisa de uma acao nova para 3+.')

    # containers
    containers = {}           # (cor, area) -> nome
    for c in doc.get('containers') or []:
        containers[(c['color'], c['at'])] = nome_container(c['color'], nome_ws(c['at']))

    # tags
    tags_cubo = {o: nome_tag_cubo(o) for o in ativos}
    tags_mesa = {area: nome_tag_mesa(info.get('reference_tag')) for area, info in stacking.items()}
    todas_tags = list(tags_cubo.values()) + sorted(set(tags_mesa.values()))

    # destino (workspace) de cada cubo que precisa se mover
    def ws_destino(o):
        tipo, area, det = fim[o]
        if tipo == 'container':
            return nome_ws(area)
        return loc_de(fim[o])

    # avisos de limitacoes do domain
    for o in ativos:
        if not parado(o) and inicio[o][0] in ('shelf_BOTTOM', 'shelf_TOP'):
            avisos.append(f'cubo {o} comeca na prateleira e precisa sair de la, mas o domain '
                          'nao tem acao para pegar cubo de prateleira (get_cube_table exige cube-at).')
        if not parado(o) and inicio[o][0] == 'container':
            avisos.append(f'cubo {o} comeca num container e precisa sair de la: sem acao no domain.')
        if fim[o][0] == 'slot':
            avisos.append(f'cubo {o} comeca fora da PP e termina num furo dela: o domain nao tem '
                          'acao put/get_cube_precision, entao esse destino fica FORA do goal.')
        if inicio[o][0] in ('slot', 'pp_desconhecido') and fim[o][0] != 'slot':
            avisos.append(f'cubo {o} comeca na PP e termina fora dela: ele entra no PDDL como '
                          'cube-at <pp>, mas a rotina de precisao pode nao libera-lo.')
    if any(t in ('shelf_BOTTOM', 'shelf_TOP') for t in [c[0] for c in fim.values()]):
        avisos.append('BOTTOM e TOP da prateleira compartilham o mesmo waypoint: o PDDL nao distingue '
                      'o nivel (goal e\' so on-top-shelf no workspace).')
    if internos:
        avisos.append(f'PP: cubos {internos} ficam so com a rotina de precisao (fora do PDDL); '
                      'o goal da PP e\' apenas (precision-started).')

    # init
    init = [f'(robot-at {LOC_START})', '(arm-empty)', '(base-free)']
    init.append('(vision-ready)' if INICIO_VISION_READY else '(arm-folded)')
    init.append('')
    for a, b in itertools.permutations(workspaces, 2):
        if a != LOC_FINISH:
            init.append(f'(connected {a} {b})')
    init.append('')
    if ativos:
        init += [f'(empty-slot {s})' for s in SLOTS_ROBO] + ['']

    for w in prateleiras:
        init.append(f'(shelf {w})')
    for area, tag in tags_mesa.items():
        init.append(f'(stacking-area {nome_ws(area)})')
        init.append(f'(tag-desk {nome_ws(area)} {tag})')
    for (cor, area), nome in containers.items():
        init.append(f'(container-at {nome} {nome_ws(area)})')
    init.append('')

    for o in ativos:
        c = nome_cubo(o)
        tipo, area, det = inicio[o]
        init.append(f'(tag-cube {c} {tags_cubo[o]})')
        if tipo == 'container':
            init.append(f'(cube-in {c} {containers[(det, area)]})')
        elif tipo in ('shelf_BOTTOM', 'shelf_TOP'):
            init.append(f'(on-top-shelf {c} {loc_de(inicio[o])})')
        else:  # table, slot, pp_desconhecido
            init.append(f'(cube-at {c} {loc_de(inicio[o])})')
            init.append(f'(free {c})')
    init.append('')

    for o in ativos:
        if parado(o):
            continue
        c = nome_cubo(o)
        init.append(f'(target {c} {ws_destino(o)})')
        if fim[o][0] == 'container':
            _, area, cor = fim[o]
            init.append(f'(target-container {c} {containers[(cor, area)]})')

    # goal
    goal = []
    for o in ativos:
        tipo, area, det = fim[o]
        c = nome_cubo(o)
        if tipo == 'container':
            goal.append(f'(cube-in {c} {containers[(det, area)]})   ; {area} container {COR_PT[det]}')
        elif tipo in ('shelf_BOTTOM', 'shelf_TOP'):
            goal.append(f'(on-top-shelf {c} {loc_de(fim[o])})   ; {area} {tipo.split("_")[1]}')
        elif tipo == 'slot':
            continue  # sem acao put_cube_precision: ver aviso
        elif tipo == 'stack':
            continue  # abaixo
        else:
            goal.append(f'(cube-at {c} {loc_de(fim[o])})   ; {area}')
    for area in stacking:
        pilha = (doc['finish_state'].get(area) or {}).get('stack') or []
        loc = nome_ws(area)
        if len(pilha) == 1:
            goal.append(f'(cube-at {nome_cubo(pilha[0])} {loc})   ; {area} pilha de 1 cubo')
        for base, topo in zip(pilha, pilha[1:]):
            goal.append(f'(cube-stacked-top {nome_cubo(topo)} {nome_cubo(base)} {loc})'
                        f'   ; {area} pilha: {topo} sobre {base}')
    if pp_ativas:
        goal.append(f'(precision-started)   ; {", ".join(pp_ativas)}: liga a visao da mesa de precisao')
    if com_finish:
        goal.append(f'(robot-at {LOC_FINISH})')

    # texto
    cab = [
        f'; Gerado por tools/yaml_to_pddl.py a partir de {task} ({doc.get("name", "")})',
        f'; duracao da prova: {doc.get("duration_min", "?")} min',
        ';',
        '; Areas do YAML -> workspaces do PDDL:',
    ]
    for area, locs in mapa_areas.items():
        cab.append(f';   {area:6s} -> {", ".join(locs)}')
    cab.append(';')
    cab.append('; Cubos (tag -> cor): ' + ', '.join(f'{o}={COR_PT[c]}' for o, c in sorted(objetos.items())))
    for area, info in stacking.items():
        cab.append(f'; Zona de empilhamento: {area} ({nome_ws(area)}), tag de referencia {info.get("reference_tag")}')
    destinos_pp = {o: f'zone{fim[o][2]}' for o in sorted(objetos) if fim[o][0] == 'slot'}
    if destinos_pp:
        cab.append(';')
        cab.append('; Furo de destino de cada cubo (para o executor):')
        cab.append(f';   ZONA_DO_CUBO = {destinos_pp}')
    for a in avisos:
        cab.append('; AVISO: ' + a)

    def bloco(linhas, ind):
        return '\n'.join((ind + l) if l else '' for l in linhas)

    obj_linhas = [' '.join(workspaces) + ' - workspace']
    if ativos:
        obj_linhas.append(' '.join(nome_cubo(o) for o in ativos) + ' - cube')
    if todas_tags:
        obj_linhas.append(' '.join(todas_tags) + ' - tag')
    if ativos:
        obj_linhas.append(' '.join(SLOTS_ROBO) + ' - slot')
    if containers:
        obj_linhas.append(' '.join(containers.values()) + ' - container')

    texto = '\n'.join(cab) + f'''

(define (problem {task.lower()}-instance)
    (:domain salamander-domain)

    (:objects
{bloco(obj_linhas, ' ' * 8)}
    )

    (:init
{bloco(init, ' ' * 8)}
    )

    (:goal
        (and
{bloco(goal, ' ' * 12)}
        )
    )
)
'''
    return texto, workspaces, avisos


def conferir_locations_yaml(caminho, workspaces):
    """Confere workspaces contra `waypoints` e contra as chaves *_coords do arquivo."""
    with open(caminho) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    waypoints = set(params.get('waypoints', []))
    chaves = set(params)
    sem_waypoint = [w for w in workspaces if w not in waypoints]
    # w4 -> w4_coords, w4_mid_coords, w4_dir_coords, w4_esq_coords...
    sem_coords = [w for w in workspaces if w in waypoints
                  and not any(k == f'{w}_coords' or (k.startswith(f'{w}_') and k.endswith('_coords'))
                              for k in chaves)]
    return sem_waypoint, sem_coords


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('yaml', help='arquivo de equipe (.yaml) do @Work Commander')
    ap.add_argument('-o', '--saida', help='arquivo .pddl de saida (padrao: pddl/problem-<task>.pddl)')
    ap.add_argument('--locations', help='arquivo de waypoints/coordenadas (ros__parameters.waypoints) para conferir os workspaces')
    ap.add_argument('--sem-finish', action='store_true', help='nao exige (robot-at locfinish) no goal')
    args = ap.parse_args()

    with open(args.yaml) as f:
        doc = yaml.safe_load(f)
    try:
        texto, locations, avisos = traduzir(doc, com_finish=not args.sem_finish)
    except ErroContrato as e:
        sys.exit(f'ERRO: {e}')

    saida = args.saida or os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pddl',
                                       f'problem-{doc["task_id"].lower()}.pddl')
    with open(saida, 'w') as f:
        f.write(texto)
    print(f'Escrito: {os.path.normpath(saida)}')
    for a in avisos:
        print('AVISO:', a)
    if args.locations:
        sem_wp, sem_co = conferir_locations_yaml(args.locations, locations)
        if sem_wp:
            print(f'AVISO: fora de `waypoints` em {args.locations}: {", ".join(sem_wp)}')
        if sem_co:
            print(f'AVISO: waypoints sem coordenadas (*_coords): {", ".join(sem_co)}')


if __name__ == '__main__':
    main()