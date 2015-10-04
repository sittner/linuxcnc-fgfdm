//
//    Copyright (C) 2015 Sascha Ittner <sascha.ittner@modusoft.de>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

#ifndef _FGFDM_RTAPI_KMOD_H_
#define _FGFDM_RTAPI_KMOD_H_

#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/sched.h>

#define fgfdm_zalloc(size) kzalloc(size, GFP_KERNEL)
#define fgfdm_free(ptr) kfree(ptr)

#define fgfdm_gettimeofday(x) do_gettimeofday(x) 

#define FGFDM_MS_TO_TICKS(x) (HZ * x / 1000)
#define fgfdm_get_ticks() ((long) jiffies)

#define fgfdm_schedule() schedule()

#endif

