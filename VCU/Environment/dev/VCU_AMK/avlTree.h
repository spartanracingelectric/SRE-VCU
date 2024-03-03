/*****************************************************************************
 * avlTree.h - CAN message manager
 * Initial Author: Rusty P
 ******************************************************************************
 * Deals with CAN objects and its calculations in memory
 ****************************************************************************/

#ifndef AVLTREE_H_INCLUDED
#define AVLTREE_H_INCLUDED

#include "IO_Driver.h"

typedef struct AVLNode
{
    //Message Metadata -----------------------------------------------------
    //int data;
    //ubyte4 id;           /**< ID for CAN communication             */
    ubyte1 data[8];

    ubyte4 timeBetweenMessages_Min; //Fastest rate at which messages will be sent
    ubyte4 lastMessage_timeStamp;   //Last time message was sent/received

    bool required;
    ubyte4 timeBetweenMessages_Max; //Slowest rate at which messages will be sent, OR max time between receiving messages before throwing an error

    //Tree stuff -----------------------------------------------------
    //struct AVLNode*  left;
    //struct AVLNode*  right;
    //int      height;
} AVLNode;

//Note on passing arrays: http://stackoverflow.com/questions/5573310/difference-between-passing-array-and-array-pointer-into-function-in-c
AVLNode *AVL_insert(AVLNode **t, ubyte4 messageID, ubyte1 messageData[8], ubyte4 timeBetweenMessages_Min, ubyte4 timeBetweenMessages_Max, bool required);

//http://www.zentut.com/c-tutorial/c-avl-tree/

#endif // AVLTREE_H_INCLUDED