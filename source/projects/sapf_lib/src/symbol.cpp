//    SAPF - Sound As Pure Form
//    Copyright (C) 2019 James McCartney
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "symbol.hpp"
#include "VM.hpp"
#include "Hash.hpp"
#include <string.h>
#include <atomic>

const int kSymbolTableSize = 4096;
const int kSymbolTableMask = kSymbolTableSize - 1;
	
// global atomic symbol table
volatile std::atomic<String*> sSymbolTable[kSymbolTableSize];



static String* SymbolTable_lookup(String* list, const char* name, int32_t hash)
{
	while (list) {
		if (list->hash == hash && strcmp(list->s, name) == 0)
			return list;
		list = list->nextSymbol;
	}
	return nullptr;
}

static String* SymbolTable_lookup(const char* name, int hash)
{
	return SymbolTable_lookup(sSymbolTable[hash & kSymbolTableMask].load(), name, hash);
}

static String* SymbolTable_lookup(const char* name)
{
	uintptr_t hash = Hash(name);
	return SymbolTable_lookup(name, (int)hash);
}

P<String> getsym(const char* name)
{
	// thread safe

	int32_t hash = Hash(name);
    int32_t binIndex = hash & kSymbolTableMask;
	volatile std::atomic<String*>* bin = &sSymbolTable[binIndex];
	while (1) {
        // get the head of the list.
		String* head = bin->load();
        // search the list for the symbol
		String* existingSymbol = head;
		while (existingSymbol) {
			if (existingSymbol->hash == hash && strcmp(existingSymbol->s, name) == 0) {
				return existingSymbol;
			}
			existingSymbol = existingSymbol->nextSymbol;
		}
		String* newSymbol = new String(name, hash, head);
		if (bin->compare_exchange_weak(head, newSymbol)) {
			newSymbol->retain(); 
			return newSymbol;
		}
        delete newSymbol;
	}	
}
