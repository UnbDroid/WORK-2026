'''#!/usr/bin/env python3
"""
Coordenador da prova (automático).

1. Espera o PlanSys2 carregar o problema (gerado pelo yaml_to_pddl.py).
2. Pede o PLANO ao planejador do PlanSys2 (nada escrito à mão).
3. Fatia o plano: cada fatia termina num (move ...). Assim o executor recebe
   planos pequenos, que ele executa sem travar.
4. Executa as fatias em sequência.
5. Se uma fatia falhar, pede um plano NOVO a partir do estado atual e continua
   (até 'max_replans' vezes).

Parâmetros:
  slice_mode   : "move" (padrão; corta depois de cada move) ou "fixed"
  slice_size   : nº de ações por fatia quando slice_mode = "fixed" (padrão 8)
  max_replans  : quantas vezes replaneja depois de uma falha (padrão 2)
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import Plan, PlanItem
from plansys2_msgs.srv import GetDomain, GetPlan, GetProblem, GetProblemInstances


class ChallengeCoordinator(Node):
    def __init__(self):
        super().__init__('challenge_coordinator')
        self.declare_parameter('slice_mode', 'move')
        self.declare_parameter('slice_size', 8)
        self.declare_parameter('max_replans', 2)
        self.slice_mode = self.get_parameter('slice_mode').value
        self.slice_size = int(self.get_parameter('slice_size').value)
        self.max_replans = int(self.get_parameter('max_replans').value)

        self.exec_client = ActionClient(self, ExecutePlan, 'execute_plan')
        self.instances_cli = self.create_client(GetProblemInstances, 'problem_expert/get_problem_instances')
        self.domain_cli = self.create_client(GetDomain, 'domain_expert/get_domain')
        self.problem_cli = self.create_client(GetProblem, 'problem_expert/get_problem')
        self.plan_cli = self.create_client(GetPlan, 'planner/get_plan')

    # ---------- utilitários ----------
    def call(self, client, req, timeout=None):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def wait_for_problem(self):
        self.get_logger().info('Aguardando o PlanSys2 carregar o problema...')
        for cli in (self.instances_cli, self.domain_cli, self.problem_cli, self.plan_cli):
            cli.wait_for_service()
        while rclpy.ok():
            res = self.call(self.instances_cli, GetProblemInstances.Request())
            if res is not None and len(res.instances) > 0:
                self.get_logger().info(f'Problema carregado: {len(res.instances)} objetos.')
                time.sleep(1.0)
                return
            time.sleep(0.5)

    def get_plan(self):
        """Pede ao planejador um plano a partir do estado ATUAL do problema."""
        domain = self.call(self.domain_cli, GetDomain.Request())
        problem = self.call(self.problem_cli, GetProblem.Request())
        if not (domain and domain.success and problem and problem.success):
            self.get_logger().error('Não consegui ler o domínio/problema do PlanSys2.')
            return None
        req = GetPlan.Request()
        req.domain = domain.domain
        req.problem = problem.problem
        self.get_logger().info('Pedindo plano ao planejador...')
        res = self.call(self.plan_cli, req)
        if res is None or not res.success or len(res.plan.items) == 0:
            info = res.error_info if res is not None else 'sem resposta'
            self.get_logger().error(f'Planejador não encontrou plano: {info}')
            return None
        self.get_logger().info(f'Plano com {len(res.plan.items)} ações:')
        for it in res.plan.items:
            self.get_logger().info(f'   {it.action}')
        return [it.action for it in res.plan.items]

    def slice_plan(self, actions):
        fatias, atual = [], []
        for a in actions:
            atual.append(a)
            if self.slice_mode == 'fixed':
                if len(atual) >= self.slice_size:
                    fatias.append(atual)
                    atual = []
            elif a.strip('() ').startswith('move '):
                fatias.append(atual)
                atual = []
        if atual:
            fatias.append(atual)
        return fatias

    def run_slice(self, idx, total, acoes):
        self.get_logger().info(f'--- FATIA {idx}/{total} ({len(acoes)} ações) ---')
        plan = Plan()
        t = 0.0
        for a in acoes:
            item = PlanItem()
            item.time = t
            item.action = a
            item.duration = 1.0
            plan.items.append(item)
            t += 0.001
        goal = ExecutePlan.Goal()
        goal.plan = plan

        send = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'Executor rejeitou a fatia {idx}.')
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        res = result.result()
        ok = res is not None and res.status == GoalStatus.STATUS_SUCCEEDED and res.result.success
        if ok:
            self.get_logger().info(f'--- FATIA {idx}/{total} CONCLUÍDA ---')
        else:
            self.get_logger().error(f'Fatia {idx} falhou.')
        return ok

    # ---------- fluxo principal ----------
    def run(self):
        self.get_logger().info('Aguardando o executor do PlanSys2...')
        self.exec_client.wait_for_server()
        self.wait_for_problem()

        replans = 0
        while rclpy.ok():
            acoes = self.get_plan()
            if acoes is None:
                return False
            fatias = self.slice_plan(acoes)
            falhou = False
            for i, f in enumerate(fatias, 1):
                if not self.run_slice(i, len(fatias), f):
                    falhou = True
                    break
            if not falhou:
                self.get_logger().info('PROVA CONCLUÍDA: todas as fatias executadas!')
                return True
            if replans >= self.max_replans:
                self.get_logger().error('Limite de replanejamentos atingido. Parando.')
                return False
            replans += 1
            self.get_logger().warn(f'Replanejando a partir do estado atual ({replans}/{self.max_replans})...')
            time.sleep(1.0)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ChallengeCoordinator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()'''
