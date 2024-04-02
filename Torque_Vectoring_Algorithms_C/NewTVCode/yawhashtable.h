#include <stdio.h>
#include <stdlib.h>
#include <math.h> // Include math.h for NAN constant

#define TABLE_SIZE 100

// Define a structure for the hash table entry
typedef struct HashEntry {
    int key1;
    int key2;
    float value;
    struct HashEntry* next; // For chaining in case of collisions
} HashEntry;

// Define a structure for the hash table itself
typedef struct {
    HashEntry* entries[TABLE_SIZE]; // Array of pointers to hash entries
} HashTable;

// Static constant hashtable
static const HashTable STATIC_HASH_TABLE = {{NULL}}; // Initialized with all pointers set to NULL

// Hash function
int hash(int key1, int key2) {
    // A simple hash function combining key1 and key2
    return (key1 + key2) % TABLE_SIZE;
}

// Create a new hash table
HashTable* createHashTable() {
    HashTable* table = (HashTable*)malloc(sizeof(HashTable));
    if (table == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(EXIT_FAILURE);// not for vcu 
    }
    for (int i = 0; i < TABLE_SIZE; i++) {
        table->entries[i] = NULL;
    }
    return table;
}

// Insert a key-value pair into the hash table
void insert(HashTable* table, int key1, int key2, float value) {
    int index = hash(key1, key2);
    
    // Create a new entry
    HashEntry* entry = (HashEntry*)malloc(sizeof(HashEntry));
    if (entry == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(EXIT_FAILURE);
    }
    entry->key1 = key1;
    entry->key2 = key2;
    entry->value = value;
    entry->next = NULL;
    
    // Handle collisions by chaining
    if (table->entries[index] != NULL) {
        HashEntry* current = table->entries[index];
        while (current->next != NULL) {
            current = current->next;
        }
        current->next = entry;
    } else {
        table->entries[index] = entry;
    }
}

// Retrieve a value from the hash table
float get(HashTable* table, int key1, int key2) {
    int index = hash(key1, key2);
    HashEntry* entry = table->entries[index];
    while (entry != NULL) {
        if (entry->key1 == key1 && entry->key2 == key2) {
            return entry->value;
        }
        entry = entry->next;
    }
    return NAN; // Key not found, return NaN saftey feature dont use NAN 
}

// Initialize the hash table with values from the YAWlookup table
void initializeYawHashTable(HashTable* table) {
    float yawLookupTable[12][7] = {
        // x-axis: velocity in increments of 5(m/s)
    // y-axis: steering in increments of 5 
    //   5  ,  10   , 15  , 20, , 25  , 30 ,  35
        {0.49, 1.00, 1.48, 1.95, 2.43, 2.93, 3.66},
        {1.48, 1.99, 2.97, 3.93, 4.87, 3.17, 3.69},
        {1.52, 2.98, 4.44, 5.87, 7.35, 8.74, 10.20},
        {2.01, 3.97, 5.90, 7.81, 9.86, 11.775, 13.74},
        {2.45, 4.97, 7.41, 9.81, 12.26, 14.72, 17.17},
        {2.98, 5.96, 8.86, 11.78, 14.67, 17.60, 20.53},
        {3.47, 6.91, 10.36, 13.92, 17.42, 20.90, 24.38},
        {4.00, 7.93, 11.84, 15.87, 19.78, 23.74, 27.69},
        {4.53, 8.90, 13.29, 17.52, 21.90, 26.28, 30.66},
        {4.92, 9.91, 14.70, 19.56, 24.45, 29.34, 34.23},
        {5.41, 10.89, 16.18, 21.54, 26.93, 32.31, 37.695},
        {6.02, 11.78, 17.78, 23.52, 29.40, 35.28, 41.16},

    };
    for (int row = 0; row < 12; row++) {
        for (int column = 0; column < 7; column++) {
            int velocity = (row + 1) * 5;
            int steering = (column + 1) * 5;
            float value = yawLookupTable[row][column];
            insert(table, velocity, steering, value);
        }
    }
}

// Destroy the hash table and free allocated memory
void destroyHashTable(HashTable* table) {
    for (int i = 0; i < TABLE_SIZE; i++) {
        HashEntry* current = table->entries[i];
        while (current != NULL) {
            HashEntry* next = current->next;
            free(current); 
            current = next;
        }
    }
    free(table);
}

// Main function
/*
int main() {
    // Create a hash table and initialize it with values from the lookup table
    HashTable* table = createHashTable();
    initializeHashTable(table);
    
    // Retrieve values
    printf("Value for (velocity = %d, steering = %d): %.2f\n", 5, 5, get(table, 5, 5));
    
    // Destroy the hash table
    destroyHashTable(table);

    return 0;

}
*/
