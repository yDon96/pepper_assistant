# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


# This is a simple example for a custom action which utters "Hello World!"

from typing import Any, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import AllSlotsReset, SlotSet
from rasa_sdk.forms import FormValidationAction 
import psycopg2
from configparser import ConfigParser
from database_local.config import configDB


def addProductDB(user, product,quantity):
    params = configDB()
    connection = psycopg2.connect(**params)
    cursor = connection.cursor()

    postgres_insert_query = """ INSERT INTO  shopping_list (nome, prodotto, quantità) VALUES (%s,%s,%s)"""
    record_to_insert = (user, product, quantity)
    cursor.execute(postgres_insert_query, record_to_insert)

    connection.commit()
    count = cursor.rowcount
    print(count, "Record inserted successfully into shopping list table")

    # closing database connection.
    if connection:
        cursor.close()
        connection.close()
        print("PostgreSQL connection is closed")

def updateProductDB(user, product,quantity):
    params = configDB()
    connection = psycopg2.connect(**params)
    cursor = connection.cursor()
    postgres_insert_query = """ UPDATE shopping_list SET quantità= %s where (nome=%s and prodotto=%s)"""
    record_to_update = (quantity, user, product)
    cursor.execute(postgres_insert_query, record_to_update)

    connection.commit()
    count = cursor.rowcount
    print(count, "Record update successfully into shopping list table")

    # closing database connection.
    if connection:
        cursor.close()
        connection.close()
        print("PostgreSQL connection is closed")



def viewListDB(dispatcher, tracker):
        
        try:
            params = configDB()
# Connect to the PostgreSQL database
            #conn = psycopg2.connect(host="localhost", port = 5432, database="postgres", user="postgres", password="Armstrong981")
            conn = psycopg2.connect(**params)
            # Create a new cursor
            cur = conn.cursor()

            current_shoppinglist = next(tracker.get_latest_entity_values("shopping_type"), None)
            current_user = next(tracker.get_latest_entity_values("user"), None)
            #Restituisce NONE se non trova l'entità.
            if not current_shoppinglist:
                msg = f"I don't understand. Can you repeat?"
                dispatcher.utter_message(text=msg)
                return []
            if current_shoppinglist :
                # msg = f"Ok, I found an entity shopping list!"
                # dispatcher.utter_message(text=msg)

                     #questo lo printa sul terminale "Rasa run actions"
                    msg = f"In your shopping list are:\n"
                    dispatcher.utter_message(text=msg)
                    sql_Query = "select prodotto, quantità from shoppingList where nome =%s"
                    nome = (str(current_user), )
                    cur = conn.cursor()
                    cur.execute(sql_Query, nome)
                    records = cur.fetchall()
                    for record in records:
                        print(f"{record[1]} {record[0]}")
                        msg3 = f"{record[1]} {record[0]}"
                        dispatcher.utter_message(text=msg3)

                 
        except (Exception, psycopg2.Error) as error:
            print("Error while fetching data from PostgreSQL", error)
        finally:
            if conn:
                cur.close()
                conn.close()    
                print("PostgreSQL connection is closed")
         
    
def removeElemDB(dispatcher,tracker):

   
    try:
        params = configDB()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()
        name = str(next(tracker.get_latest_entity_values("user"),None))
        product = str(next(tracker.get_latest_entity_values("product"),None))
        cur.execute("""delete from shopping_list where (nome = %s and prodotto = %s)""",(name,product))
        sql_Query = "select * from shoppingList where (nome = %s and prodotto = %s)",(name,product)
        nome = (str(name), )
        cur = conn.cursor()
        cur.execute(sql_Query, nome)
        records = cur.fetchall()
        if product not in records[1]:
            msg4 = f"The product is not present in the list"
            dispatcher.utter_message(text=msg4)
        else:
            print(name+" "+product)
            conn.commit()
            msg3 = f"The product has been removed!"
            dispatcher.utter_message(text=msg3)
    except(Exception,psycopg2.Error) as err:
        print("Operation failed with error:",err)

    finally:
        cur.close()
        conn.close()

def updateListDB(dispatcher, tracker):
    quantity_to_update = tracker.get_slot("quantity")
    product_to_update = tracker.get_slot("product")
    user_to_update=next(tracker.get_latest_entity_values("user"), None)
    print("user, prod, quant", user_to_update, product_to_update, quantity_to_update)  
    if not product_to_update:
        msg= f"I don't understand. Can you repeat the product that i want to update?"
        dispatcher.utter_message(text=msg)
    if product_to_update:
        msg1= f"Ok, the product that you want update is: " + str(product_to_update)
        dispatcher.utter_message(text=msg1)
        try:
            updateProductDB(user_to_update, product_to_update, quantity_to_update)
            msg2 = f"Ok the element has been update!"  
            dispatcher.utter_message(text=msg2)
        except (Exception, psycopg2.Error) as error:        
            print("Failed to update record into shopping list table", error)
            msg3 = f"The product is not present, attention!"
            dispatcher.utter_message(text=msg3)
            msg4 = f"Do you want to do something else?"
            dispatcher.utter_message(text=msg4)

class ActionRemove(Action):
    def name(self)->Text:
        return "action_remove"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        removeElemDB(dispatcher,tracker,domain)
        return [AllSlotsReset()]
    
        

class ActionAdd(Action):

    def name(self) -> Text:
        return "action_add"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        product = tracker.get_slot("product")
        quantity = tracker.get_slot("quantity")
        user = next(tracker.get_latest_entity_values("user"), None)

        print("\nUser:", user)
        print("\nProduct:",product)
        print("\nQuantity:",quantity)

        try: 
            addProductDB(dispatcher, str(user), str(product), str(quantity))
            dispatcher.utter_message(f"Thanks, your answers have been recorded!")
        except (Exception, psycopg2.Error) as error:        
            print("Failed to insert record into shoppinglist table", error)
            # if the element is alredy present
            if str(error.pgcode) == "23505":            
                dispatcher.utter_message(f"The product is already in your list. Select option 'update' to update the product's quantity or 'remove' to remove it.")  
        finally:        
            return [AllSlotsReset()]

        
class ActionView(Action):
    def name(self)->Text:
        return "action_view"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        viewListDB(dispatcher,tracker,domain)
        return []




class ActionUpdate(Action):
    def name(self)->Text:
        return "action_update"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        updateListDB(dispatcher,tracker,domain)
        return [AllSlotsReset()]