import tkinter as tk
from serving_turtle.kitchen.database_handler import DatabaseHandler


class KitchenGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.db_handler = DatabaseHandler()  # DB 핸들러 객체 생성

        # GUI 초기 설정
        self.root = tk.Tk()
        self.root.title("Kitchen Node")
        self.root.geometry("800x600")

        # 주문 내역 표시
        self.order_list = tk.Listbox(self.root, font=("Helvetica", 12))
        self.order_list.pack(pady=10, fill=tk.BOTH, expand=True)

        # 버튼들
        self.accept_button = tk.Button(self.root, text="수락", command=self.accept_order, state=tk.DISABLED)
        self.accept_button.pack(pady=10)

        self.reject_button = tk.Button(self.root, text="거절", command=self.reject_order, state=tk.DISABLED)
        self.reject_button.pack(pady=10)

        self.today_sales_button = tk.Button(
            self.root, text="오늘 매출 조회", command=self.view_today_sales
        )
        self.today_sales_button.pack(pady=10)

        self.today_menu_sales_button = tk.Button(
            self.root, text="오늘 메뉴별 판매량 조회", command=self.view_today_menu_sales
        )
        self.today_menu_sales_button.pack(pady=10)

    def update_status(self, message, color):
        """GUI 상태 업데이트"""
        self.order_list.insert(tk.END, message)
        self.order_list.itemconfig(tk.END, {'fg': color})
    
    def add_order(self, order_text, table_id, menu_item, quantity):
        """주문을 리스트에 추가하고 버튼 활성화"""
        menu_items = menu_item.split(", ")  # ✅ 쉼표로 구분된 메뉴 항목
        for item in menu_items:
            menu_name, qty = item.split(" x")
            self.db_handler.save_order(table_id, menu_name.strip(), int(qty.strip()))
        self.order_list.insert(tk.END, order_text)
        self.enable_buttons()

    def accept_order(self):
        """주문 수락"""
        selected_index = self.order_list.curselection()  # 선택된 항목의 인덱스 가져오기
        if not selected_index:
            return  # 선택되지 않은 경우 아무 작업도 하지 않음

        selected = self.order_list.get(selected_index)  # 선택된 항목 텍스트 가져오기

        # 이미 처리된 주문은 상태를 변경하지 않음
        if selected.endswith("- Accepted") or selected.endswith("- Rejected"):
            return
        self.ros_node.accept_pending_goal(selected)

        table_id = int(selected.split(" ")[1].replace(":", ""))  # 테이블 ID 추출
        self.ros_node.publish_status(selected, 'Accepted')  # ROS 노드에 수락 상태 전송
        self.db_handler.update_order_status(table_id, 'Accepted')  # DB에 상태 업데이트

        # 텍스트 상태 업데이트 (기존 글자를 수정)
        updated_text = f"{selected} - Accepted"
        self.order_list.delete(selected_index)  # 기존 항목 삭제
        self.order_list.insert(selected_index, updated_text)  # 업데이트된 항목 삽입
        self.order_list.itemconfig(selected_index, {'fg': 'green'})  # 항목의 색상 변경


    def reject_order(self):
        """주문 거절"""
        selected_index = self.order_list.curselection()  # 선택된 항목의 인덱스 가져오기
        if not selected_index:
            return  # 선택되지 않은 경우 아무 작업도 하지 않음

        selected = self.order_list.get(selected_index)  # 선택된 항목 텍스트 가져오기

        # 이미 처리된 주문은 상태를 변경하지 않음
        if selected.endswith("- Accepted") or selected.endswith("- Rejected"):
            return

        table_id = int(selected.split(" ")[1].replace(":", ""))  # 테이블 ID 추출
        self.ros_node.publish_status(selected, 'Rejected')  # ROS 노드에 거절 상태 전송
        self.db_handler.update_order_status(table_id, 'Rejected')  # DB에 상태 업데이트

        # 텍스트 상태 업데이트 (기존 글자를 수정)
        updated_text = f"{selected} - Rejected"
        self.order_list.delete(selected_index)  # 기존 항목 삭제
        self.order_list.insert(selected_index, updated_text)  # 업데이트된 항목 삽입
        self.order_list.itemconfig(selected_index, {'fg': 'red'})  # 항목의 색상 변경

    def view_today_sales(self):
        """오늘 매출 조회"""
        total_sales = self.db_handler.view_today_sales()
        self.order_list.insert(tk.END, f"오늘 매출: {total_sales}원")

    def view_today_menu_sales(self):
        """오늘 메뉴별 판매량 조회"""
        menu_sales = self.db_handler.view_today_menu_sales()
        self.order_list.insert(tk.END, "오늘 메뉴별 판매량:")
        for menu, quantity in menu_sales.items():
            self.order_list.insert(tk.END, f"{menu}: {quantity}개")

    def enable_buttons(self):
        self.accept_button.config(state=tk.NORMAL)
        self.reject_button.config(state=tk.NORMAL)

    def disable_buttons(self):
        self.accept_button.config(state=tk.DISABLED)
        self.reject_button.config(state=tk.DISABLED)

    def run(self):
        self.root.mainloop()