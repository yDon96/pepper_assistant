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
from database_local.config import config
import psycopg2
#ciao


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

    
    ''''  
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
            dispatcher.utter_message(text=msg4)'''

    try:
        params = config()
        conn = psycopg2.connect(**params)
        cur = conn.cursor()
        name = str(next(tracker.get_latest_entity_values("user"),None))
        product = str(next(tracker.get_latest_entity_values("product"),None))
        cur.execute("""delete from shopping_list where (nome = %s and prodotto = %s)""",(name,product))
        #query_results = cur.fetchall()
        print(name+" "+product)
        conn.commit()
        msg3 = f"The product has been removed!"
        dispatcher.utter_message(text=msg3)
    except(Exception,psycopg2.Error) as err:
        print("Operation failed with error:",err)

    finally:
        cur.close()
        conn.close()

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