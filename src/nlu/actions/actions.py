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

import psycopg2
import pandas as pd

# Import the 'config' funtion from the config.py file

#ciao
from configparser import ConfigParser
from database_local.config import config


def print_inventory(dict):
    for item, amount in dict.items():
        print("{} ({})".format(item, amount))
    
shopping_list ={}

def viewList(dispatcher, tracker, domain):
        
        try:
            params = config()
# Connect to the PostgreSQL database
            #conn = psycopg2.connect(host="localhost", port = 5432, database="postgres", user="postgres", password="Armstrong981")
            conn = psycopg2.connect(**params)
            # Create a new cursor
            cur = conn.cursor()

            current_shoppinglist = next(tracker.get_latest_entity_values("shopping_type"), None)
            current_user = next(tracker.get_latest_entity_values("user"), None)
            #Restituisce NONE se non trova l'entità.
            if not current_shoppinglist:
                msg = f"Ops, I not found the entity shopping list!"
                dispatcher.utter_message(text=msg)
                return []
            if current_shoppinglist :
                # msg = f"Ok, I found an entity shopping list!"
                # dispatcher.utter_message(text=msg)

                     #questo lo printa sul terminale "Rasa run actions"
                    msg = f"Your shopping list is:\n"
                    dispatcher.utter_message(text=msg)
                    msg2= f"_______________________________________________"
                    dispatcher.utter_message(text=msg2)
                    sql_Query = "select prodotto, quantità from shoppingList where nome =%s"
                    nome = (str(current_user), )
                    cur = conn.cursor()
                    cur.execute(sql_Query, nome)
                    record = cur.fetchall()
                    print(record)
                    msg1= f"_______________________________________________"
                    dispatcher.utter_message(text=msg1)

                 
        except (Exception, psycopg2.Error) as error:
            print("Error while fetching data from PostgreSQL", error)
        finally:
            if conn:
                cur.close()
                conn.close()    
                print("PostgreSQL connection is closed")
         
    
def removeElem(dispatcher,tracker,domain):
    
    
    global shopping_list
    product_to_remove = next(tracker.get_latest_entity_values("product"),None)
    
    if not product_to_remove:
        msg= f"Ops, I can't find the entity product!"
        dispatcher.utter_message(text=msg)
    if product_to_remove:
        msg1= f"Ok, the product that you want remove is: " + str(product_to_remove)
        dispatcher.utter_message(text=msg1)
        if product_to_remove in shopping_list:
            del shopping_list[product_to_remove]
            msg2 = f"Ok the element has been removed!"  
            dispatcher.utter_message(text=msg2)
        else:
            msg3 = f"The product is not present, attention!"
            dispatcher.utter_message(text=msg3)
            msg4 = f"Do you want to do something else?"
            dispatcher.utter_message(text=msg4)

def updateList(dispatcher, tracker,domain):
    global shopping_list
    
    quantity_to_update = tracker.get_slot("quantity")
    product_to_update = tracker.get_slot("product")
    print("prod,quant", product_to_update,quantity_to_update)  
    if not product_to_update:
        msg= f"Ops, I not found the entity product!"
        dispatcher.utter_message(text=msg)
    
    if product_to_update:
        msg1= f"Ok, the product that you want update is: " + str(product_to_update)
        dispatcher.utter_message(text=msg1)
        if product_to_update in shopping_list:
            shopping_list[product_to_update]=quantity_to_update
            msg2 = f"Ok the element has been update!"  
            dispatcher.utter_message(text=msg2)
        else:
            msg3 = f"The product is not present, attention!"
            dispatcher.utter_message(text=msg3)
            msg4 = f"Do you want to do something else?"
            dispatcher.utter_message(text=msg4)

class ActionRemoveItem(Action):
    def name(self)->Text:
        return "action_remove"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        removeElem(dispatcher,tracker,domain)
        return [AllSlotsReset()]
    
        

class ActionSubmit(Action):

    def name(self) -> Text:
        return "action_add"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        product = tracker.get_slot("product")
        quantity = tracker.get_slot("quantity")
        if(product in shopping_list):
            dispatcher.utter_message(f"The product is already in your list. Select option 'update' to update the product's quantity or 'remove' to remove it.")
            return [AllSlotsReset()]
        print("\nProduct:",product)
        print("Quantity:",quantity)
        dispatcher.utter_message(f"Thanks, your answers have been recorded!")
        shopping_list[product]=quantity
        return [AllSlotsReset()]

        
class ActionView(Action):
    def name(self)->Text:
        return "action_view"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        viewList(dispatcher,tracker,domain)
        return []

class ActionRemoveView(Action):
    def name(self)->Text:
        return "action_remove_view"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        global shopping_list
        msg1= f"_______________________________________________"
        dispatcher.utter_message(text=msg1)
        for key,value in shopping_list.items():
            dispatcher.utter_message(text=f"{key}\t{value}")
        msg2= f"_______________________________________________"
        dispatcher.utter_message(text=msg2)
        if len(shopping_list)==0:
            msg6 = f"Pay attention! The list is empty."
            dispatcher.utter_message(text=msg6)
        msg5 = f"Which product you want to remove from list?"
        dispatcher.utter_message(text=msg5)
        return []

class ActionUpdateView(Action):
    def name(self)->Text:
        return "action_update_view"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        global shopping_list
        for key,value in shopping_list.items():
            dispatcher.utter_message(text=f"{key}\t{value}")
        if len(shopping_list)==0:
            msg6 = f"Pay attention! There aren't product in the list."
            dispatcher.utter_message(text=msg6)
        return []

class ActionUpdate(Action):
    def name(self)->Text:
        return "action_update"       
    def run(self, dispatcher: CollectingDispatcher, 
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        updateList(dispatcher,tracker,domain)
        return [AllSlotsReset()]