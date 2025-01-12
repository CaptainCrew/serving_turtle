import tkinter as tk
from tkinter import ttk
#from database_handler import DatabaseHandler

class KitchenGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        #self.db_handler = DatabaseHandler()

        self.root = tk.Tk()
        self.root.title("Kitchen Node")
        self.root.geometry("800x600")

        self.order_list = tk.Listbox(self.root, font=("Helvetica", 12))
        self.order_list.pack(pady=10, fill=tk.BOTH, expand=True)

        self.accept_button = tk.Button(self.root, text="수락", command=self.accept_order, state=tk.DISABLED)
        self.accept_button.pack(pady=5)

        self.reject_button = tk.Button(self.root, text="거절", command=self.reject_order, state=tk.DISABLED)
        self.reject_button.pack(pady=5)
    def add_order(self, order_text):
        """✅ 주문을 리스트에 추가하고 버튼 활성화"""
        self.order_list.insert(tk.END, order_text)
        self.accept_button.config(state=tk.NORMAL)
        self.reject_button.config(state=tk.NORMAL)
    def accept_order(self):
        selected = self.order_list.get(tk.ACTIVE)
        self.ros_node.publish_status(selected, 'Accepted')
        self.accept_button.config(state=tk.DISABLED)

    def reject_order(self):
        selected = self.order_list.get(tk.ACTIVE)
        self.ros_node.publish_status(selected, 'Rejected')
        self.reject_button.config(state=tk.DISABLED)

    def run(self):
        self.root.mainloop()
