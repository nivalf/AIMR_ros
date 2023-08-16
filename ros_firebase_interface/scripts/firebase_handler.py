# firebase_handler.py

import firebase_admin
from firebase_admin import credentials, firestore

class FirebaseHandler:
    """
    Handler for Firebase operations including adding, updating, and retrieving documents.
    """

    def __init__(self, cred_path):
        """
        Initialize the FirebaseHandler instance with provided credentials.

        Args:
            cred_path (str): Path to Firebase credentials file.
        """
        try:
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred)
            self.db = firestore.client()
        except Exception as e:
            raise ValueError(f"Failed to initialize Firebase: {e}")

    def add_document(self, collection_name, data):
        """
        Add a new document to the specified collection.

        Args:
            collection_name (str): Name of the collection in Firebase.
            data (dict): Document data to be added.

        Returns:
            DocumentReference: Reference to the Firestore document.
        """
        try:
            collection_ref = self.db.collection(collection_name)
            time, doc_ref = collection_ref.add(data)  # Extract DocumentReference from the returned tuple 
            return doc_ref
        except Exception as e:
            raise ValueError(f"Failed to add document: {e}")

    def update_document(self, doc_ref, updates):
        """
        Update an existing document with provided data.

        Args:
            doc_ref (DocumentReference): Reference to the Firestore document.
            updates (dict): Data updates for the document.
        """
        try:
            doc_ref.update(updates)
        except Exception as e:
            raise ValueError(f"Failed to update document: {e}")
