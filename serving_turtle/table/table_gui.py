import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import os

IMAGES_DIR = "/home/rokey/turtlebot3_ws/src/serving_turtle/images"

MENU = {
    "Cheese Burger": {"price": 10000, "image": os.path.join(IMAGES_DIR, "cheese_burger.png")},
    "Fries": {"price": 8000, "image": os.path.join(IMAGES_DIR, "fries.png")},
    "Beer": {"price": 5000, "image": os.path.join(IMAGES_DIR, "beer.png")},
    "Tissue": {"price": 0, "image": os.path.join(IMAGES_DIR, "tissue.png")},
    "Water": {"price": 0, "image": os.path.join(IMAGES_DIR, "water.png")},
}

class TableGUI:
    def __init__(self, action_client):
        self.action_client = action_client  # ✅ Action 클라이언트 연결

        self.root = tk.Tk()
        self.root.title("Table Node - Restaurant Order System")
        self.root.geometry("800x600")

        self.menu_frame = tk.Frame(self.root)
        self.menu_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.menu_widgets = {}
        row, col = 0, 0

        for menu_item, details in MENU.items():
            frame = tk.Frame(self.menu_frame, relief="ridge", borderwidth=2)
            frame.grid(row=row, column=col, padx=10, pady=10)

            img = Image.open(details["image"]).resize((100, 100))
            photo = ImageTk.PhotoImage(img)
            img_label = tk.Label(frame, image=photo)
            img_label.image = photo
            img_label.pack()

            name_label = tk.Label(frame, text=f"{menu_item}\n{details['price']}원", font=("Helvetica", 12))
            name_label.pack()

            quantity_spinbox = ttk.Spinbox(frame, from_=0, to=10, width=5, font=("Helvetica", 12))
            quantity_spinbox.insert(0, "0")
            quantity_spinbox.pack(pady=5)

            self.menu_widgets[menu_item] = quantity_spinbox

            col += 1
            if col >= 3:
                col = 0
                row += 1

        self.order_button = tk.Button(self.root, text="주문하기", font=("Helvetica", 14, "bold"), bg="#4CAF50",
                                      fg="white", command=self.publish_order)
        self.order_button.pack(pady=20)

        self.status_label = tk.Label(self.root, text="대기 중...", font=("Helvetica", 12), fg="blue")
        self.status_label.pack()

    def publish_order(self):
        order_details = []
        total_quantity = 0

        for menu_item, spinbox in self.menu_widgets.items():
            try:
                quantity = int(spinbox.get())
                if quantity > 0:
                    order_details.append(f"{menu_item} x{quantity}")
                    total_quantity += quantity
            except ValueError:
                continue

        if not order_details:
            self.update_status("수량을 입력하세요.", "red")
            return

        self.action_client.send_order(order_details, total_quantity)
        self.update_status("주문을 전송했습니다.", "blue")

    def update_status(self, message, color):
        """상태 라벨 업데이트"""
        self.status_label.config(text=message, fg=color)

    def run(self):
        self.root.mainloop()
