#ifndef _COMPAT_MTKFB_H_
#define _COMPAT_MTKFB_H_
#include <linux/compat.h>

#ifdef CONFIG_COMPAT

struct compat_fb_overlay_layer {
	compat_uint_t layer_id;
	compat_uint_t layer_enable;

	compat_uptr_t src_base_addr;
	compat_uptr_t src_phy_addr;
	compat_uint_t src_direct_link;
	compat_int_t src_fmt;
	compat_uint_t src_use_color_key;
	compat_uint_t src_color_key;
	compat_uint_t src_pitch;
	compat_uint_t src_offset_x, src_offset_y;
	compat_uint_t src_width, src_height;

	compat_uint_t tgt_offset_x, tgt_offset_y;
	compat_uint_t tgt_width, tgt_height;
	compat_int_t layer_rotation;
	compat_int_t layer_type;
	compat_int_t video_rotation;

	compat_uint_t isTdshp;	/* set to 1, will go through tdshp first, then layer blending, then to color */

	compat_int_t next_buff_idx;
	compat_int_t identity;
	compat_int_t connected_type;
	compat_uint_t security;
	compat_uint_t alpha_enable;
	compat_uint_t alpha;
	compat_int_t fence_fd;	/* 8135 */
	compat_int_t ion_fd;	/* 8135 CL 2340210 */
};

#define COMPAT_MTKFB_SET_OVERLAY_LAYER			MTK_IOW(0, struct compat_fb_overlay_layer)
#define COMPAT_MTKFB_SET_VIDEO_LAYERS			MTK_IOW(2, struct compat_fb_overlay_layer)
#define COMPAT_MTKFB_CAPTURE_FRAMEBUFFER		MTK_IOW(3, compat_ulong_t)
#define COMPAT_MTKFB_GET_POWERSTATE				MTK_IOR(21, compat_ulong_t)
#define COMPAT_MTKFB_AEE_LAYER_EXIST            MTK_IOR(23, compat_ulong_t)
#define COMPAT_MTKFB_FACTORY_AUTO_TEST          MTK_IOR(25, compat_ulong_t)
#define COMPAT_MTKFB_META_RESTORE_SCREEN		MTK_IOW(101, compat_ulong_t)

#endif

#endif /*_COMPAT_MTKFB_H_*/
