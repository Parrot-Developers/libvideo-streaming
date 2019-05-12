/**
 * Copyright (c) 2016 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _VSTRM_EVENTS_H_
#define _VSTRM_EVENTS_H_


/* Event type
 * Note: for sender/receiver compatibility reasons, values can only be added
 * here, not removed; values should be explicit; VSTRM_EVENT_MAX must be less
 * than or equal to UINT8_MAX (events are serialized as 8bit integers) */
enum vstrm_event {
	/* No event (should never be sent) */
	VSTRM_EVENT_NONE = 0,

	/* Pipeline reconfiguration (e.g. when changing the video recording
	 * framerate or switching between photo and video recording modes) */
	VSTRM_EVENT_RECONFIGURE = 1,

	/* Resolution change, which is not a whole pipeline reconfiguration
	 * (e.g. when spatial scalability changes the stream resolution) */
	VSTRM_EVENT_RESOLUTION_CHANGE = 2,

	/* A photo is being taken (to be sent as close as possible to the real
	 * shutter trigger) */
	VSTRM_EVENT_PHOTO_TRIGGER = 3,

	/* Element count */
	VSTRM_EVENT_MAX,
};


/**
 * Get an enum vstrm_event value from a string.
 * Valid strings are only the suffix of the event name (eg. 'RECONFIGURE').
 * The case is ignored.
 * @param str: event value to convert
 * @return the enum vstrm_event value or VSTRM_EVENT_NONE if unknown
 */
VSTRM_API enum vstrm_event vstrm_event_from_str(const char *str);


/**
 * Get a string from an enum vstrm_event value.
 * @param event: event value to convert
 * @return a string description of the event
 */
VSTRM_API const char *vstrm_event_to_str(enum vstrm_event event);


#endif /* !_VSTRM_EVENTS_H_ */
