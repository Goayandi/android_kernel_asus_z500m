/*
 * drives/usb/pd/pd_policy_engine_drs.c
 * Power Delvery Policy Engine for DRS
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
 * [PD2.0] Figure 8-49: Type-C DFP to UFP Data Role Swap State Diagram
 */

void pe_drs_dfp_ufp_evaluate_dr_swap_entry(
			pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_dpm_drs_evaluate_swap(pd_port, PD_ROLE_UFP);
	pd_free_pd_event(pd_port, pd_event);
}

void pe_drs_dfp_ufp_accept_dr_swap_entry(
			pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_send_ctrl_msg(pd_port, TCPC_TX_SOP, PD_CTRL_ACCEPT);
}

void pe_drs_dfp_ufp_change_to_ufp_entry(
			pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_dpm_drs_change_role(pd_port, PD_ROLE_UFP);
	pd_free_pd_event(pd_port, pd_event);
}

void pe_drs_dfp_ufp_send_dr_swap_entry(pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_send_ctrl_msg(pd_port, TCPC_TX_SOP, PD_CTRL_DR_SWAP);
}

void pe_drs_dfp_ufp_reject_dr_swap_entry(
				pd_port_t *pd_port, pd_event_t *pd_event)
{
	if (pd_event->msg_sec == PD_DPM_NAK_REJECT)
		pd_send_ctrl_msg(pd_port, TCPC_TX_SOP, PD_CTRL_REJECT);
	else
		pd_send_ctrl_msg(pd_port, TCPC_TX_SOP, PD_CTRL_WAIT);
}

/*
 * [PD2.0] Figure 8-50: Type-C UFP to DFP Data Role Swap State Diagram
 */

void pe_drs_ufp_dfp_evaluate_dr_swap_entry(
			pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_dpm_drs_evaluate_swap(pd_port, PD_ROLE_DFP);
	pd_free_pd_event(pd_port, pd_event);
}

void pe_drs_ufp_dfp_accept_dr_swap_entry(
			pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_send_ctrl_msg(pd_port, TCPC_TX_SOP, PD_CTRL_ACCEPT);
}

void pe_drs_ufp_dfp_change_to_dfp_entry(
			pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_dpm_drs_change_role(pd_port, PD_ROLE_DFP);
	pd_free_pd_event(pd_port, pd_event);
}

void pe_drs_ufp_dfp_send_dr_swap_entry(pd_port_t *pd_port, pd_event_t *pd_event)
{
	pd_send_ctrl_msg(pd_port, TCPC_TX_SOP, PD_CTRL_DR_SWAP);
}

void pe_drs_ufp_dfp_reject_dr_swap_entry(
			pd_port_t *pd_port, pd_event_t *pd_event)
{
	if (pd_event->msg_sec == PD_DPM_NAK_REJECT)
		pd_send_ctrl_msg(pd_port, TCPC_TX_SOP, PD_CTRL_REJECT);
	else
		pd_send_ctrl_msg(pd_port, TCPC_TX_SOP, PD_CTRL_WAIT);
}
