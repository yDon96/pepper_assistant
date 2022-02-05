# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


from typing import Any, Text, Dict, List
from numpy import product
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import AllSlotsReset, SlotSet
from rasa_sdk.forms import FormValidationAction 
import psycopg2
from configparser import ConfigParser
from database_local.config import configDB

def addProductDB(dispatcher, tracker):
    record=[]
    product = tracker.get_slot("product")
    quantity = tracker.get_slot("quantity")
    user = next(tracker.get_latest_entity_values("user"), None)

    print("\nUser:", user)
    print("\nProduct:",product)
    print("\nQuantity:",quantity)
    
    if not product:
        msg= f"I don't understand. Can you repeat?"
        dispatcher.utter_message(text=msg)
    if product:
        msg1= "Ok, the product that you want insert is: " + str(product)
        print(msg1)
        params = configDB()
        connection = psycopg2.connect(**params)
        cursor = connection.cursor()
        postgres_insert_query = """ INSERT INTO  shopping_list (nome, prodotto, quantità) VALUES (%s,%s,%s)"""
        record_to_insert = (user, product, quantity)
        cursor.execute(postgres_insert_query, record_to_insert)
        record.extend([product, quantity])
        connection.commit()
        count = cursor.rowcount
        print(count, "Record insert successfully into shopping list table")
    # closing database connection.
    if connection:
        cursor.close()
        connection.close()
        print("PostgreSQL connection is closed")
    return record

def addProduct(dispatcher, tracker):
    try:
        record=addProductDB(dispatcher, tracker)
        msg2="Thanks,inserted successfully. Your answers have been recorded!"    
        dispatcher.utter_message(template="utter_update_list", text=msg2, product=record, status="add") 
    except (Exception, psycopg2.Error) as error:        
        print("Failed to insert record into shoppinglist table", error)
        # if the element is alredy present
        if str(error.pgcode) == "23505":            
            dispatcher.utter_message(f"The product is already in your list, you can update the product's quantity or remove it.")


def updateProductDB(dispatcher, tracker):
    quantity_to_update = tracker.get_slot("quantity")
    product_to_update = tracker.get_slot("product")
    user_to_update=next(tracker.get_latest_entity_values("user"), None)
    print("user, prod, quant", user_to_update, product_to_update, quantity_to_update)  
    if not product_to_update:
        msg= f"I don't understand. Can you repeat?"
        dispatcher.utter_message(text=msg)
    if product_to_update:
        msg1= "Ok, the product that you want update is: " + str(product_to_update)
        print(msg1)
    params = configDB()
    connection = psycopg2.connect(**params)
    cursor = connection.cursor()
    postgres_insert_query = """ UPDATE shopping_list SET quantità= %s where (nome=%s and prodotto=%s)"""
    record_to_update = (quantity_to_update, user_to_update, product_to_update)
    cursor.execute(postgres_insert_query, record_to_update)
    record=[]
    record.extend([product, quantity)
    connection.commit()
    count = cursor.rowcount
    print(count, "Record update successfully into shopping list table")
    # closing database connection.
    if connection:
        cursor.close()
        connection.close()
        print("PostgreSQL connection is closed")
    return record

def updateProduct(dispatcher, tracker):
    try:
        record=updateProductDB(dispatcher, tracker)
        msg2 = f"Ok the quantity of the product has been updated!"      
        dispatcher.utter_message(template="utter_update_list", text=msg2, product=record, status="update") 
    except (Exception, psycopg2.Error) as error:        
        print("Failed to update record into shopping list table", error)
        msg3 = f"The product is not present! Do you want to do something else?"
        dispatcher.utter_message(text=msg3)
        


def viewListDB(dispatcher, tracker):
    current_shoppinglist = next(tracker.get_latest_entity_values("shopping_type"), None)
    current_user = next(tracker.get_latest_entity_values("user"), None)

    #Restituisce NONE se non trova l'entità.
    # if not current_shoppinglist:
    #     msg = f"I don't understand. Can you repeat?"
    #     dispatcher.utter_message(text=msg)
    #     return []
    # if current_shoppinglist :
        # msg = f"Ok, I found an entity shopping list!"
        # dispatcher.utter_message(text=msg)
        #questo lo printa sul terminale "Rasa run actions"
        #             
    current_shoppinglist = next(tracker.get_latest_entity_values("shopping_type"), None)
    current_user = next(tracker.get_latest_entity_values("user"), None)
    params = configDB()
    conn = psycopg2.connect(**params)
    # Create a new cursor
    cur = conn.cursor()
    sql_Query = "select prodotto, quantità from shopping_list where nome =%s"
    nome = (str(current_user), )
    cur = conn.cursor()
    cur.execute(sql_Query, nome)
    records=cur.fetchall()
    if conn:
        cur.close()
        conn.close()    
        print("PostgreSQL connection is closed")
    return records

def viewList(dispatcher, tracker):
    try:
        #dispatcher.utter_message(text=msg)
        #for record in records:
            #  print(f"{record[1]} {record[0]}")
            #  msg3 = f"{record[1]} {record[0]}"
            #  dispatcher.utter_message(text=msg3) 
        records =  viewListDB(dispatcher, tracker)   
        if len(records)==0:
            msg4 = f"Your list is empty"
            dispatcher.utter_message(text=msg4)
        else:
            msg = f"Your shopping list is shown on my tablet"
            dispatcher.utter_message(template="utter_view_list", text=msg, product=records, status="view")  
    except (Exception, psycopg2.Error) as error:
        print("Error while fetching data from PostgreSQL", error)            
          

def removeElemDB(dispatcher,tracker):   
    record=[]
    name = str(next(tracker.get_latest_entity_values("user"),None))
    product = str(next(tracker.get_latest_entity_values("product"),None))
    if not product:
        msg= f"I don't understand. Can you repeat?"
        dispatcher.utter_message(text=msg)
    if product:
        msg1= "Ok, the product that you want remove is: " + str(product)
        print(msg1)
    params = configDB()
    conn = psycopg2.connect(**params)
    cur = conn.cursor()
    sql_Query = "select * from shopping_list where (nome = %s and prodotto = %s)"
    cur = conn.cursor()
    cur.execute(sql_Query,(name,product))
    records = cur.fetchall()
    if len(records)==0:
        msg4 = f"The product is not present in the list"
        dispatcher.utter_message(text=msg4)
    else:
        record.extend([product, records[0][2]])
        print(name+" "+product)
        cur.execute("""delete from shopping_list where (nome = %s and prodotto = %s)""",(name,product))
        conn.commit()
        cur.close()
        conn.close()
    return record

def removeElem(dispatcher,tracker):
    try:
       record=removeElemDB(dispatcher,tracker)
       # dispatcher.utter_message(text=msg3) 
       msg3 = f"The product has been removed!"              
       dispatcher.utter_message(template="utter_update_list", text=msg3, product=record, status="remove")
    except(Exception,psycopg2.Error) as err:
        print("Operation failed with error:",err)

#----------------------------------------------------------------------------------------------------------------------------------------------

class ActionRemove(Action):
    def name(self)->Text:
        return "action_remove"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        removeElem(dispatcher,tracker)
        return [AllSlotsReset()]
    
class ActionAdd(Action):
    def name(self) -> Text:
        return "action_add"
    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        addProduct(dispatcher, tracker)      
        return [AllSlotsReset()]
      
class ActionView(Action):
    def name(self)->Text:
        return "action_view"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        viewList(dispatcher,tracker)
        return []

class ActionUpdate(Action):
    def name(self)->Text:
        return "action_update"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        updateProduct(dispatcher,tracker)
        return [AllSlotsReset()]

