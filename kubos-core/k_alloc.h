/*
 * KubOS Core Flight Services
 * Copyright (C) 2016 Kubos Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @defgroup Allocator
 * @addtogroup Allocator
 * @{
 */

 /**
  *
  * @file       k_alloc.h
  * @brief      Buffer Allocator Wrapper
  *
  * @author     kubos.co
  */


#ifndef K_ALLOC_H
#define K_ALLOC_H

#include <stdlib.h>

typedef struct k_alloc {
    void * (* _new)(size_t size);
    void * (* _realloc)(void * buff, size_t old_size, size_t new_size);
    void (* _free)();
} k_alloc_t;

#endif

/* @} */
