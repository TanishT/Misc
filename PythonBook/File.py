#!/usr/bin/python3

import pymysql

db = pymysql.connect("localhost", "root", "", "users")

cursor = db.cursor()

cursor.execute("SELECT VERSION()")

data = cursor.fetchone()

print(data)

db.close()
