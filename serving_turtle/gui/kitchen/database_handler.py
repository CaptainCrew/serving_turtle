import sqlite3

DB_PATH = "/home/djkim/project1_ws/src/week1.db"

def connect_to_db():
    connection = sqlite3.connect(DB_PATH, check_same_thread=False)
    cursor = connection.cursor()
    return connection, cursor

class DatabaseHandler:
    def __init__(self):
        self.connection, self.cursor = connect_to_db()

    def save_order(self, table_id, menu_item, quantity):
        self.cursor.execute("SELECT menu_id FROM Menu WHERE name = ?", (menu_item,))
        result = self.cursor.fetchone()
        if result:
            menu_id = result[0]
            self.cursor.execute("""
                INSERT INTO Orders (table_id, menu_id, quantity, status)
                VALUES (?, ?, ?, 'Pending')
            """, (table_id, menu_id, quantity))
            self.connection.commit()

    def update_order_status(self, table_id, status):
        self.cursor.execute("""
            UPDATE Orders SET status = ?
            WHERE table_id = ? AND status = 'Pending'
        """, (status, table_id))
        self.connection.commit()
