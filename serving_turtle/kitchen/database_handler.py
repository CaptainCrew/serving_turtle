import sqlite3
from datetime import date

DB_PATH = "/home/rokey/turtlebot3_ws/src/serving_turtle/serving_turtle/kitchen/week1.db"


class DatabaseHandler:
    def __init__(self):
        self.connection = sqlite3.connect(DB_PATH, check_same_thread=False)
        self.cursor = self.connection.cursor()

    def save_order(self, table_id, menu_item, quantity):
        """주문 저장"""
        today_date = date.today().strftime('%Y-%m-%d')
        self.cursor.execute("SELECT menu_id FROM Menu WHERE name = ?", (menu_item,))
        result = self.cursor.fetchone()
        if result:
            menu_id = result[0]
            self.cursor.execute("""
                INSERT INTO Orders (table_id, menu_id, quantity, status, date)
                VALUES (?, ?, ?, 'Pending', ?)
            """, (table_id, menu_id, quantity, today_date))
            self.connection.commit()

    def update_order_status(self, table_id, status):
        """주문 상태 업데이트"""
        self.cursor.execute("""
            UPDATE Orders SET status = ?
            WHERE table_id = ? AND status = 'Pending'
        """, (status, table_id))
        self.connection.commit()

    def view_today_sales(self):
        """오늘 매출 조회"""
        today_date = date.today().strftime('%Y-%m-%d')
        self.cursor.execute("""
            SELECT SUM(o.quantity * m.price) AS total_sales
            FROM Orders o
            JOIN Menu m ON o.menu_id = m.menu_id
            WHERE o.status = 'Accepted' AND o.date = ?
        """, (today_date,))
        result = self.cursor.fetchone()
        return result[0] if result and result[0] else 0

    def view_today_menu_sales(self):
        """오늘 메뉴별 판매량 조회"""
        today_date = date.today().strftime('%Y-%m-%d')
        self.cursor.execute("""
            SELECT m.name, SUM(o.quantity) AS total_quantity
            FROM Orders o
            JOIN Menu m ON o.menu_id = m.menu_id
            WHERE o.status = 'Accepted' AND o.date = ?
            GROUP BY m.name
        """, (today_date,))
        results = self.cursor.fetchall()
        return {name: quantity for name, quantity in results}