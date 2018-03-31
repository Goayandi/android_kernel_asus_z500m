#ifndef __CMDQ_FENCE_H__
#define __CMDQ_FENCE_H__

/*
**	public functions
*/
#define MTK_FB_INVALID_FENCE_FD (-1)
/*
**	return 0 if success;
*/
int32_t cmdqFenceGetFence(struct cmdqFenceStruct *pFence);

/*
**	return release status
**	0 : success
**	-1 : release fail
*/
int32_t cmdqFenceReleaseFence(struct cmdqFenceStruct fence);

/*
**	create timeline for fence
**	return 0 for success, -1 for fail
*/
int32_t cmdqFenceCreateTimeLine(void);

#endif
