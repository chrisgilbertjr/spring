
#ifndef SP_LINKED_LIST_H
#define SP_LINKED_LIST_H

/// remove a node from a linked list
#define SP_LINKED_LIST_REMOVE(type, node, list)                               \
    spAssert(list != NULL, "Error: list is NULL, cannot remove");             \
    spAssert(node != NULL, "Error: node is NULL, cannot remove");             \
    type* n_prev = node->prev;                                                \
    type* n_next = node->next;                                                \
    node->prev = NULL;                                                        \
    node->next = NULL;                                                        \
                                                                              \
    if (n_prev != NULL && n_next != NULL)                                     \
    {                                                                         \
        n_prev->next = n_next;                                                \
        n_next->prev = n_prev;                                                \
    }                                                                         \
                                                                              \
    else if (n_prev == NULL && n_next == NULL)                                \
    {                                                                         \
        list = NULL;                                                          \
    }                                                                         \
                                                                              \
    else if (n_prev == NULL)                                                  \
    {                                                                         \
        list = n_next;                                                        \
        n_next->prev = NULL;                                                  \
    }                                                                         \
                                                                              \
    else if (n_next == NULL)                                                  \
    {                                                                         \
        n_prev->next = NULL;                                                  \
    }                                                                         \
                                                                              \
    else                                                                      \
    {                                                                         \
        spAssert(spFalse, "removing ... shouldnt reach this...");               \
    }                                                                         \

/// prepend a node to a linked list
#define SP_LINKED_LIST_PREPEND(type, node, list)                              \
    spAssert(node != NULL, "Error: node is NULL, cannot add body");           \
                                                                              \
    if (list == NULL)                                                         \
    {                                                                         \
        node->next = NULL;                                                    \
    }                                                                         \
    else                                                                      \
    {                                                                         \
        list->prev = node;                                                    \
        node->next = list;                                                    \
    }                                                                         \
    node->prev = NULL;                                                        \
    list = node;                                                              \

#endif