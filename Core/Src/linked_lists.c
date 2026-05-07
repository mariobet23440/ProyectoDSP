/*
 * LIBRERÍA PARA LISTAS ENLAZADAS
 * linked_lists.c
 *
 *  Creada el: 6/05/2026
 *  Author: Mario Alejandro Betancourt Franco
 */

#include "linked_lists.h"

// Dado que

// Función para crear un nodo nuevo
struct Node* createNode(float data){
	struct Node* newNode = (struct Node*) malloc(sizeof(struct Node));

	if(!newNode){
		// Memory allocation error;
		return NULL;
	}

	newNode->data = data;
	newNode->next = NULL;
	return newNode;
}

// Insertar un nodo al inicio
void insertAtBeginning(struct Node** headRef, float data)
{
	struct Node* newNode = createNode(data);
	newNode->next = *headRef;
	*headRef = newNode;
}
