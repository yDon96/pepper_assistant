# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


# This is a simple example for a custom action which utters "Hello World!"

from operator import contains
from typing import Any, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import AllSlotsReset, SlotSet
from rasa_sdk.forms import FormValidationAction
#ciao 
import psycopg2

def addproductDB(dispatcher, user, product,quantity):
   
        connection = psycopg2.connect(host="localhost", port = 5432, database="CR", user="cr", password="postgress")
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




def print_inventory(dict):
    for item, amount in dict.items():
        print("{} ({})".format(item, amount))
    
shopping_list ={}

def viewList(dispatcher, tracker, domain):
        current_shoppinglist = next(tracker.get_latest_entity_values("shopping_type"), None)
        global shopping_list
         #Restituisce NONE se non trova l'entità.
        if not current_shoppinglist:
            msg = f"Ops, I not found the entity shopping list!"
            dispatcher.utter_message(text=msg)
            return []
        if current_shoppinglist :
            # msg = f"Ok, I found an entity shopping list!"
            # dispatcher.utter_message(text=msg)
            if(len(shopping_list)!=0):
                print_inventory(shopping_list) #questo lo printa sul terminale "Rasa run actions"
                msg = f"Your shopping list is:\n"
                dispatcher.utter_message(text=msg)
                msg2= f"_______________________________________________"
                dispatcher.utter_message(text=msg2)
                for key,value in shopping_list.items():
                    dispatcher.utter_message(text=f"{key}\t{value}")
                msg1= f"_______________________________________________"
                dispatcher.utter_message(text=msg1)
		        
            if (len(shopping_list)==0):
                msg = f"Are you sure you have already added an item in the list!!?\n"
                dispatcher.utter_message(text=msg)
                msg1= f"I think no. Please restart the conversation with command: /restart or do another action.\n"
                dispatcher.utter_message(text=msg1)
         
    
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
        user = next(tracker.get_latest_entity_values("user"), None)

        print("\nUser:", user)
        print("\nProduct:",product)
        print("\nQuantity:",quantity)

        try: 
            addproductDB(dispatcher, str(user), str(product), str(quantity))
            dispatcher.utter_message(f"Thanks, your answers have been recorded!")
        except (Exception, psycopg2.Error) as error:        
            print("Failed to insert record into mobile table", error)
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
