/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef __KVX_SB_COOPERATIVE_H__
#define __KVX_SB_COOPERATIVE_H__

/* Position of the ToC Image Header flag in the header */
#define TOC_IMAGE_FLAG_OFFSET 0

/* ToC Image Header Flags (most significant bytes are set to 1) */
enum toc_image_header_flag {
	LEGACY_IMG     = 0xFFFF00F0u, /* Legacy image */
	CORRUPTED_IMG  = 0xFFFF0330u, /* Corrupted image */
	CURRENT_IMG    = 0xFFFF0FF0u, /* Current image */
	RECOVERY_IMG   = 0xFFFF03F0u, /* Recovery image */
	VALID_IMG      = 0xFFFF0FFCu, /* Validated image (cooperative boot) */
	UNDER_EVAL_IMG = 0xFFFF0FFFu, /* Under evaluation image */
	NEW_IMG        = 0xFFFF3FFFu, /* New image */
	NO_IMG         = 0xFFFFFFFFu, /* Empty slot */
};

#endif
