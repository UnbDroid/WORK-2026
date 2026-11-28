#!/usr/bin/env bash
# Uso:  ./rodar_prova.sh /media/<pendrive>/ATT1.yaml          (terminal mostra só o essencial)
#       ./rodar_prova.sh /media/<pendrive>/ATT1.yaml --tudo   (terminal mostra tudo)
# O log COMPLETO sempre vai para ~/logs_prova/prova_<data>.log
set -o pipefail

YAML="$1"
MODO="$2"
if [ -z "$YAML" ] || [ ! -f "$YAML" ]; then
  echo "Uso: $0 <arquivo_da_prova.yaml> [--tudo]"; exit 1
fi
case "$YAML" in
  *.yaml|*.yml) ;;
  *) echo "[ERRO] Passe o YAML dos juízes (.yaml), não o '$YAML'."; exit 1 ;;
esac

WS=~/droid/WORK-2026/work_ws
PKG=$WS/src/salamander_planning
CONVERSOR=$PKG/launch/yaml_to_pddl.py
CENARIO=$PKG/config/cenario_2.yaml
PROBLEMA=$PKG/pddl/problem-prova.pddl

LOGDIR=~/logs_prova
mkdir -p "$LOGDIR"
LOG=$LOGDIR/prova_$(date +%Y%m%d_%H%M%S).log

VERDE='\033[1;32m'; VERM='\033[1;31m'; AMAR='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "${VERDE}[OK]${NC} $1"; }
erro() { echo -e "${VERM}[ERRO]${NC} $1"; echo -e "     Log completo: $LOG"; exit 1; }

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

# ---------- ETAPA 1: conversão ----------
echo "==> [1/3] Convertendo $(basename "$YAML") em PDDL..."
[ -f "$CONVERSOR" ] || erro "Conversor não encontrado em $CONVERSOR"
[ -f "$CENARIO" ]   || erro "Cenário não encontrado em $CENARIO"
if python3 "$CONVERSOR" "$YAML" -o "$PROBLEMA" --locations "$CENARIO" >>"$LOG" 2>&1; then
  ok "Problema gerado: $PROBLEMA"
else
  tail -n 15 "$LOG"
  erro "Falha na conversão do YAML (mensagem acima)."
fi

# ---------- ETAPA 2: Nav2 e localização estão no ar? ----------
echo "==> [2/3] Conferindo se Nav2 e localização estão rodando..."
if timeout 8 ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; then
  ok "Nav2 respondendo."
else
  erro "Nav2 não está rodando. Suba localização + Nav2 antes deste script."
fi
if timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -q "Translation"; then
  ok "TF map -> base_link existe (robô localizado)."
else
  echo -e "${AMAR}[AVISO]${NC} Sem TF map->base_link. Dê o '2D Pose Estimate' no RViz."
fi

# ---------- ETAPA 3: PlanSys2 + nós + coordenador ----------
echo "==> [3/3] Subindo PlanSys2, nós de ação e coordenador... (Ctrl+C para parar)"
echo "     Log completo: $LOG"
echo "----------------------------------------------------------------"

export RCUTILS_COLORIZED_OUTPUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1

# Só passa para a tela: mensagens do coordenador, erros, e nós que morreram
FILTRO='challenge_coordinator|ERROR|FATAL|process has died|exception|Traceback'

if [ "$MODO" == "--tudo" ]; then
  ros2 launch salamander_planning teste1.launch.py problem_file:="$PROBLEMA" 2>&1 | tee -a "$LOG"
else
  ros2 launch salamander_planning teste1.launch.py problem_file:="$PROBLEMA" 2>&1 \
    | tee -a "$LOG" | grep --line-buffered -E "$FILTRO"
fi

echo "----------------------------------------------------------------"
echo "Fim. Log completo: $LOG"
