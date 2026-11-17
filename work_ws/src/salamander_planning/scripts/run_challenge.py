#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import Plan, PlanItem
from plansys2_msgs.srv import GetProblemInstances
from action_msgs.msg import GoalStatus

class ChallengeCoordinator(Node):
    def __init__(self):
        super().__init__('challenge_coordinator')
        
        self.declare_parameter('challenge', 'att1')
        self.challenge = self.get_parameter('challenge').get_parameter_value().string_value
        
        self._action_client = ActionClient(self, ExecutePlan, 'execute_plan')
        # Cliente para ler a memória do PlanSys2 e garantir que o problema carregou
        self._get_instances_client = self.create_client(GetProblemInstances, 'problem_expert/get_problem_instances')
        
        # Variáveis de controle para o fatiamento
        self.plan_chunks = self.get_challenge_plan(self.challenge)
        self.current_chunk_idx = 0

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

    def get_challenge_plan(self, challenge_name):
        plans = {
            'att1': [
                # --- FATIA 1: Pegar os cubos 1, 2 e 3 em w1 e mover para w2 ---
                [
                    "(reset_arm start)",
                    "(move start w1)",
                    "(zero_position w1)",
                    "(align_tag w1 tag1)",
                    "(get_cube_table w1 cubo1 tag1)",
                    "(put_cube_slot cubo1 slot1 w1)",
                    "(reset_arm w1)",
                    "(move w1 w2)",
                    "(zero_position w2)",
                    "(align_tag w2 tag2)",
                    "(get_cube_table w2 cubo2 tag2)",
                    "(put_cube_slot cubo2 slot2 w2)",
                    "(reset_arm w2)",
                    "(move w2 w3)"
                ],
                # --- FATIA 2: Guardar nos containers (w2 e w3) e mover para w4 ---
                [
                    "(zero_position w3)",
                    "(align_container w3 cubo1 container-blue)",
                    "(reset_arm w3)",
                    "(get_cube_slot cubo1 slot1 w3)",
                    "(put_cube_container w3 cubo1 container-blue)",
                    "(move w3 w4)",
                    "(get_cube_slot cubo2 slot2 w4)",
                    "(put_cube_table w4 cubo2)",
                    "(zero_position w4)",
                    "(align_tag w4 tag3)",
                    "(get_cube_table w4 cubo3 tag3)",
                    "(put_cube_slot cubo3 slot1 w4)"
                ],
                # --- FATIA 3: Pegar cubo4 e cubo5 em w4, empilhar em w5 e finalizar ---
                [
                    "(reset_arm w4)",
                    "(move w4 w5)",
                    "(get_cube_slot cubo3 slot1 w5)",
                    "(put_cube_table w5 cubo3)",
                    "(move w5 finish)"
                ]
            ]
        }
        
        if challenge_name not in plans:
            return []
            
        return plans[challenge_name]

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
    
    if not node.plan_chunks:
        node.get_logger().error(f"Plano para o desafio '{node.challenge}' não encontrado!")
    else:
        node.send_next_chunk()
        
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()