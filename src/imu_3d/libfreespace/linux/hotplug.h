/*
 * This file is part of libfreespace.
 *
 * Copyright (c) 2009 Hillcrest Laboratories, Inc.
 *
 * libfreespace is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef _HOTPLUG_H_
#define _HOTPLUG_H_


/**
 * Initialize the hotplug file descriptor
 */
int freespace_hotplug_init();

/**
 * Cleanup hotplug initializations and close the hotplug
 * file descriptor
 */
void freespace_hotplug_exit();

/**
 * Return a timeout if the hotplug
 * code should be called back based on a timer
 * rather than a file descriptor activating.
 *
 * @return the time in milliseconds. Negative means wait forever
 */
int freespace_hotplug_timeout();


/**
 * Get the file descriptor for hotplug events
 * This is for use by poll or select
 */
int freespace_hotplug_getFD();

/**
 * Handle hotplug event
 * @param recheck set to one when to rescan for new devices.
 * Returns FREESPACE_SUCCESS if some kind of hotplug event occurred
 * Returns an error code otherwise
 */
int freespace_hotplug_perform(int* recheck);

#endif // _HOTPLUG_H_
