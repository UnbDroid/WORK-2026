#!/usr/bin/env python3
# Coordenador da prova.
# ALTERADO: em vez das fatias escritas à mão, ele pede o plano ao PlanSys2
# (o mesmo do "get plan" no terminal) e fatia sozinho: uma fatia termina
# a cada N ações (move ...). O resto (envio das fatias) é igual ao antigo.

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import Plan, PlanItem
from plansys2_msgs.srv import GetProblemInstances, GetDomain, GetProblem, GetPlan
from action_msgs.msg import GoalStatus


class ChallengeCoordinator(Node):
    def __init__(self):
        super().__init__('challenge_coordinator')

        # NOVO: quantos (move ...) por fatia. 1 = uma fatia por mesa visitada.
        self.declare_parameter('moves_por_fatia', 1)
        self.moves_por_fatia = max(1, self.get_parameter('moves_por_fatia').value)

        self._action_client = ActionClient(self, ExecutePlan, 'execute_plan')
        self._get_instances_client = self.create_client(GetProblemInstances, 'problem_expert/get_problem_instances')
        # NOVO: clientes para montar o plano automaticamente
        self._get_domain_client = self.create_client(GetDomain, 'domain_expert/get_domain')
        self._get_problem_client = self.create_client(GetProblem, 'problem_expert/get_problem')
        self._get_plan_client = self.create_client(GetPlan, 'planner/get_plan')

        self.plan_chunks = []
        self.current_chunk_idx = 0

    def call(self, client, req):
        client.wait_for_service()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def wait_for_problem_expert(self):
        self.get_logger().info('Aguardando a memória do Problem Expert popular (esperando arquivo de problema)...')
        self._get_instances_client.wait_for_service()

        req = GetProblemInstances.Request()
        while rclpy.ok():
            future = self._get_instances_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                if len(future.result().instances) > 0:
                    self.get_logger().info(f'Memória populada! {len(future.result().instances)} objetos detectados.')
                    time.sleep(1.0)
                    return
            time.sleep(0.5)

    # NOVO: pede o plano ao planejador do PlanSys2
    def get_plan_from_planner(self):
        domain = self.call(self._get_domain_client, GetDomain.Request())
        problem = self.call(self._get_problem_client, GetProblem.Request())
        if not domain or not domain.success or not problem or not problem.success:
            self.get_logger().error('Não consegui ler o domínio/problema do PlanSys2.')
            return []

        self.get_logger().info('Pedindo plano ao planejador (pode levar alguns segundos)...')
        req = GetPlan.Request()
        req.domain = domain.domain
        req.problem = problem.problem
        res = self.call(self._get_plan_client, req)
        if res is None or not res.success:
            info = res.error_info if res is not None else 'sem resposta'
            self.get_logger().error(f'Planejador NÃO encontrou plano: {info}')
            return []

        acoes = [item.action for item in res.plan.items]
        self.get_logger().info(f'Plano com {len(acoes)} ações:')
        for i, a in enumerate(acoes, 1):
            self.get_logger().info(f'  {i:2d}. {a}')
        return acoes

    # NOVO: fatiamento automático — corta depois de cada N "(move ...)"
    def slice_plan(self, acoes):
        fatias, atual, moves = [], [], 0
        for a in acoes:
            atual.append(a)
            if a.startswith('(move '):
                moves += 1
                if moves >= self.moves_por_fatia:
                    fatias.append(atual)
                    atual, moves = [], 0
        if atual:
            fatias.append(atual)

        self.get_logger().info(f'Plano dividido em {len(fatias)} fatias:')
        for i, f in enumerate(fatias, 1):
            self.get_logger().info(f'  FATIA {i}: {" ".join(f)}')
        return fatias

    def send_next_chunk(self):
        if self.current_chunk_idx >= len(self.plan_chunks):
            self.get_logger().info('TODAS AS FATIAS FORAM CONCLUÍDAS COM SUCESSO!')
            return

        chunk = self.plan_chunks[self.current_chunk_idx]

        self.get_logger().info(f"--- Carregando FATIA {self.current_chunk_idx + 1} de {len(self.plan_chunks)} ---")

        plan_msg = Plan()
        time_offset = 0.0

        for act in chunk:
            item = PlanItem()
            item.time = time_offset
            item.action = act
            item.duration = 1.0
            plan_msg.items.append(item)
            time_offset += 0.001

        goal_msg = ExecutePlan.Goal()
        goal_msg.plan = plan_msg

        self.get_logger().info('Injetando fatia no PlanSys2...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Erro crítico: O executor rejeitou a FATIA {self.current_chunk_idx + 1}!')
            return

        self.get_logger().info(f'SUCESSO! Fatia {self.current_chunk_idx + 1} validada. Executando...')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'--- FATIA {self.current_chunk_idx + 1} CONCLUÍDA! ---')
            self.current_chunk_idx += 1
            self.send_next_chunk()
        else:
            self.get_logger().error(f'A Fatia {self.current_chunk_idx + 1} falhou no meio da execução! Status: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = ChallengeCoordinator()

    node.get_logger().info('Aguardando o Executor do PlanSys2 ficar online...')
    node._action_client.wait_for_server()

    # Aguarda o launch carregar o PDDL no Problem Expert antes de disparar o plano!
    node.wait_for_problem_expert()

    # ALTERADO: plano vem do planejador e é fatiado automaticamente
    acoes = node.get_plan_from_planner()
    node.plan_chunks = node.slice_plan(acoes) if acoes else []

    if not node.plan_chunks:
        node.get_logger().error('Sem plano para executar. Confira o problem-prova.pddl.')
    else:
        node.send_next_chunk()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
