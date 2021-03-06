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
    product = tracker.get_slot("product")
    quantity = tracker.get_slot("quantity")
    user = next(tracker.get_latest_entity_values("user"), None)    
    if not product:
        msg= f"I don't understand. Can you repeat?"
        dispatcher.utter_message(text=msg)
    if product:
        msg1= "Ok, the product that you want insert is: " + str(product)
        print(msg1)
        params = configDB()
        connection = psycopg2.connect(**params)
        cursor = connection.cursor()
        ##added by sabrina 13:37
        sql_Query = "select quantità from shopping_list where (nome =%s and prodotto =%s)"
        cursor.execute(sql_Query, (user,product))
        records = cursor.fetchall()
        if len(records)!=0:
            quantity = int(quantity)+int(records[0][0]) #somma della quantità se il prodotto è gia presente
            postgres_insert_query = """ UPDATE shopping_list SET quantità= %s where (nome=%s and prodotto=%s)"""
            record_to_insert = (str(quantity), user, product)
            cursor.execute(postgres_insert_query, record_to_insert)
            msg3 = f"The quantity of the product has been update!"
        else:
        ######################
            postgres_insert_query = """ INSERT INTO  shopping_list (nome, prodotto, quantità) VALUES (%s,%s,%s)"""
            record_to_insert = (user, product, quantity)
            cursor.execute(postgres_insert_query, record_to_insert)
            msg3 = f"Record inserted successfully into shopping list!"
    connection.commit()
    if connection:
        cursor.close()
        connection.close()
        print("PostgreSQL connection is closed")
    return  product, quantity, msg3

def addProduct(dispatcher, tracker):      
    try:
        record=[]
        product, quantity, msg= addProductDB(dispatcher, tracker)
        record.extend([product, quantity])              
        dispatcher.utter_message(template="utter_update_list", text=msg, product=record, status="add")
    except (Exception, psycopg2.Error) as error:        
        print("Failed to insert record into shoppinglist table", error)
           

def updateProductDB(dispatcher, tracker):
    quantity_to_update = tracker.get_slot("quantity")
    product_to_update = tracker.get_slot("product")
    user_to_update=next(tracker.get_latest_entity_values("user"), None)  
    if not product_to_update:
        msg= f"I don't understand. Can you repeat?"
        dispatcher.utter_message(text=msg)
    if product_to_update:
        msg1= "Ok, the product that you want update is: " + str(product_to_update)
        print(msg1)
        params = configDB()
        connection = psycopg2.connect(**params)
        cursor = connection.cursor()
        #########
        # sql_Query = "select * from shopping_list where (nome = %s and prodotto = %s)"
        # cur = connection.cursor()
        #cur.execute(sql_Query,(user,product))
        #records = cur.fetchall()
        #print(" r : ", records)

        #if len(records)==0:
        #    msg4 = f"The product is not present in the list"
            #   dispatcher.utter_message(text=msg4)
        ##########
        postgres_insert_query = """ UPDATE shopping_list SET quantità= %s where (nome=%s and prodotto=%s)"""
        record_to_update = (quantity_to_update, user_to_update, product_to_update)
        cursor.execute(postgres_insert_query, record_to_update)
        connection.commit()
        count = cursor.rowcount
        print(count, "Record update successfully into shopping list table")
        if connection:
            cursor.close()
            connection.close()
            print("PostgreSQL connection is closed")
    return product_to_update, quantity_to_update

def updateProduct(dispatcher, tracker):
    record=[]
    try:
            product, quantity=updateProductDB(dispatcher, tracker)
            record.extend([product, quantity])
            msg2= f"Record update successfully into shopping list!"              
            dispatcher.utter_message(template="utter_update_list", text=msg2, product=record, status="update")
    except (Exception, psycopg2.Error) as error:        
            print("Failed to update record into shopping list table", error)
            msg3 = f"The product is not present!"
            dispatcher.utter_message(text=msg3)
            msg4 = f"Do you want to do something else?"
            dispatcher.utter_message(text=msg4)


def viewListDB(dispatcher, tracker):
    params = configDB()
    conn = psycopg2.connect(**params)
    cur = conn.cursor()
    current_user = next(tracker.get_latest_entity_values("user"), None)
    
    sql_Query = "select prodotto, quantità from shopping_list where nome =%s"
    nome = (str(current_user), )
    cur = conn.cursor()
    cur.execute(sql_Query, nome)
    records = cur.fetchall()
    if conn:
        cur.close()
        conn.close()    
        print("PostgreSQL connection is closed")
    if len(records)==0:
        msg4 = f"Your list is empty!"
        dispatcher.utter_message(text=msg4)
    else:
        return records
          
def viewList(dispatcher, tracker):
    try:
        records=viewListDB(dispatcher, tracker)
        msg = f"Your shopping list is shown on my tablet"
        dispatcher.utter_message(template="utter_view_list", text=msg, product=records, status="view") 
    except (Exception, psycopg2.Error) as error:
        print("Error while fetching data from PostgreSQL", error)


def removeElemDB(dispatcher,tracker):   
    name = str(next(tracker.get_latest_entity_values("user"),None))
    product = str(next(tracker.get_latest_entity_values("product"),None)) 
    quantity=None
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
            cur.execute("""delete from shopping_list where (nome = %s and prodotto = %s)""",(name,product))
            conn.commit()
            quantity=records[0][2]
        cur.close()
        conn.close()
        return product, quantity

def removeElem(dispatcher,tracker):
    record=[]
    try:
        product, quantity= removeElemDB(dispatcher,tracker)
        record.extend([product, quantity])
        msg3 = f"The product has been removed!"                            
        dispatcher.utter_message(template="utter_update_list", text=msg3, product=record, status="remove")
    except(Exception,psycopg2.Error) as err:
        print("Operation failed with error:",err)

def emptyListDB(dispatcher, tracker):
    name = str(next(tracker.get_latest_entity_values("user"),None))
    params = configDB()
    conn = psycopg2.connect(**params)
    cur = conn.cursor()
    
    sql_Query = "select * from shopping_list where nome = %s"
    cur = conn.cursor()
    cur.execute(sql_Query,(name,))
    print(2)
    records = cur.fetchall()  
    if len(records)==0:
        msg4 = f"Your list is already empty!"
       
    else:
        cur.execute("""delete from shopping_list where nome = %s """,(name,))
        conn.commit()
        msg4 = f"Your list has been deleted!" 
    
    dispatcher.utter_message(msg4)
    cur.close()
    conn.close()


def emptyList(dispatcher, tracker):
    try:
        emptyListDB(dispatcher, tracker)
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

class ActionEmpty(Action):
    def name(self)->Text:
        return "action_empty"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        emptyList(dispatcher,tracker)
        return []