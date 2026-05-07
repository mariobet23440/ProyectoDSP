/*
 * LIBRERÍA PARA LISTAS ENLAZADAS (HEADER FILE)
 * linked_lists.h
 *
 *  Creada el: 6/05/2026
 *  Author: Mario Alejandro Betancourt Franco
 */

#ifndef INC_LINKED_LISTS_H_
#define INC_LINKED_LISTS_H_

/*
 * Una lista enlazada es una estructura de datos lineal,
 * cuyos elementos no necesariamente están en posiciones continuas.
 * Los elementos individuales se conocen como nodos y están conectados
 * entre sí mediante enlaces (Guardan un puntero que apunta hacia otro nodo).
 *
 * Un nodo contiene dos cosas: un dato y un puntero que apunta hacia otro nodo.
 *
 * El primer nodo se conoce como cabeza (head) y podemos recorrer la lista completa
 * usando este nodo y sus enlaces siguientes.
 *
 *
 * La ventaja de utilizar listas enlazadas sobre arreglos tradicionales es que
 * las operaciones de inserción al inicio de la lista y eliminación al final
 * son de complejidad constante O(1). Esto quiere decir que basta con una iteración,
 * sin la necesidad de recorrer la lista completa, para hacer este tipo de cambios sobre
 * las listas.
 *
 * Esta librería implementa listas simplemente enlazadas, donde los enlaces
 * apuntan hen una sola dirección y no al revés, ni formando lazos.
 *
 * ¡En tiempo real esto ofrece un procesamiento bastante más rápido que usando arreglos!
 */

// Includes privados
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

// Estructura para nodos
struct Node
{
	float data;
	struct Node *next;
};


/*
 * A diferencia de otros lenguajes de programación, el lenguaje C
 * no conoce tal cosa como las CLASES. Por lo tanto se usan structs y los
 * constructores de clase se deben definir por separado.
 */

#endif /* INC_LINKED_LISTS_H_ */
