/*
 * drives/usb/pd/pd_policy_engine_dr.c
 * Power Delvery Policy Engine for DR
 *
 * Copyright (C) 2015 Richtek Technology Corp.
 * Author: TH <tsunghan_tasi@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


#include <linux/usb/pd_core.h>
#include <linux/usb/pd_dpm_core.h>
#include <linux/usb/tcpci.h>
#include <linux/usb/pd_policy_engine.h>

/*
 * [PD2.0]
 * Figure 8-53 Dual-Role (Source) Get Source Capabilities diagram
 * Figure 8-54 Dual-Role (Source) Give Sink Capabilities diagram
 * Figure 8-55 Dual-Role (Sink) Get Sink Capabilities State Diagram
 * Figure 8-56 Dual-Role (Sink) Give Source Capabilities State Diagram
 */

void pe_dr_src_get_source_cap_entry(pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_send_ctrl_msg(pd_port, TCPC_TX_SOP, PD_CTRL_GET_SOURCE_CAP);
}

void pe_dr_src_get_source_cap_exit(pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_disable_timer(pd_port, PD_TIMER_SENDER_RESPONSE);
	pd_dpm_dr_inform_source_cap(pd_port, pd_event);
}

void pe_dr_src_give_sink_cap_entry(pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_dpm_send_sink_caps(pd_port);
	pd_free_pd_event(pd_port, pd_event);
}

void pe_dr_snk_get_sink_cap_entry(pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_send_ctrl_msg(pd_port, TCPC_TX_SOP, PD_CTRL_GET_SINK_CAP);
}

void pe_dr_snk_get_sink_cap_exit(pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_disable_timer(pd_port, PD_TIMER_SENDER_RESPONSE);
	pd_dpm_dr_inform_sink_cap(pd_port, pd_event);
}

void pe_dr_snk_give_source_cap_entry(pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_dpm_send_source_caps(pd_port);
	pd_free_pd_event(pd_port, pd_event);
}
