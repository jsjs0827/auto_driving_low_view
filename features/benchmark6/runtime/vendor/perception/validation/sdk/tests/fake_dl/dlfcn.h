#pragma once
// Mock dynamic loader, never compiled into the X5 worker or installed runtime.
#define RTLD_NOW 2
#define RTLD_GLOBAL 256
void* dlopen(const char*,int);
void* dlsym(void*,const char*);
char* dlerror();
int dlclose(void*);
