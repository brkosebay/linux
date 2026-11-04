// SPDX-License-Identifier: GPL-2.0-only
/*
 * Twister WFI and opt-in forced clock gating. A9 has no architectural
 * retention: deep WFI resumes at RVBAR, not at the following instruction.
 * The iOS reset path also uses a secure-monitor entry registration. Until
 * Linux owns a verified reset/resume protocol, never request power-down.
 */
#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <asm/cpuidle.h>
#include <asm/cputype.h>
#include <asm/sysreg.h>

#define A9_CPU_CFG		sys_reg(3, 5, 15, 4, 0)
#define A9_CYC_OVRD		sys_reg(3, 5, 15, 5, 0)
#define A9_DEEP_SLEEP		BIT_ULL(24)
#define A9_WFI_MODE		GENMASK_ULL(25, 24)
#define A9_FORCE_CLOCK_GATE	BIT_ULL(25)
#define A9_ARCH_RETENTION	BIT_ULL(2)

static bool enable;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Register Twister idle accounting (default: off)");

static bool clock_gate;
module_param(clock_gate, bool, 0444);
MODULE_PARM_DESC(clock_gate, "Try j71t forced WFI clock gating (default: off)");

static bool probe;
module_param(probe, bool, 0444);
MODULE_PARM_DESC(probe, "Log per-CPU idle registers without enabling idle (default: off)");

static bool a9_is_twister(void)
{
	return read_cpuid_implementor() == ARM_CPU_IMP_APPLE &&
	       read_cpuid_part_number() == APPLE_CPU_PART_TSMC_TWISTER;
}

static int __cpuidle a9_idle_wfi(struct cpuidle_device *dev,
				 struct cpuidle_driver *drv, int index)
{
	/* A rejected deep configuration must not fall back to the same WFI. */
	if (!a9_is_twister())
		return -ENODEV;
	if ((read_sysreg_s(A9_CPU_CFG) & A9_DEEP_SLEEP) ||
	    (read_sysreg_s(A9_CYC_OVRD) & A9_WFI_MODE) != A9_FORCE_CLOCK_GATE)
		return -EBUSY;

	cpu_do_idle();
	return 0;
}

static int __cpuidle a9_idle_clock_gate(struct cpuidle_device *dev,
					struct cpuidle_driver *drv, int index)
{
	u64 saved;

	/* Never touch implementation registers on an unexpected CPU. */
	if (!a9_is_twister())
		return a9_idle_wfi(dev, drv, 0);

	/* A reset-based sleep configuration cannot safely use this return path. */
	if (read_sysreg_s(A9_CPU_CFG) & A9_DEEP_SLEEP)
		return a9_idle_wfi(dev, drv, 0);
	saved = read_sysreg_s(A9_CYC_OVRD);
	if (saved & BIT_ULL(24))
		return a9_idle_wfi(dev, drv, 0);
	if ((saved & A9_WFI_MODE) == A9_FORCE_CLOCK_GATE)
		return a9_idle_wfi(dev, drv, 0);

	/*
	 * j71t 20H350: 0xfffffff00718bbcc ORs bit25 to force clock gating.
	 * Bit24 would request power-down; CPU_CFG bit24 enables deep sleep.
	 * Neither is set here, and no PMGR CPU/cluster registers are written.
	 * Clock gating retains GPR/FP/EL1 state; CPU_PM save/restore is needed
	 * only by a future, separately verified context-losing retention state.
	 */
	write_sysreg_s((saved & ~A9_WFI_MODE) | A9_FORCE_CLOCK_GATE, A9_CYC_OVRD);
	isb();
	cpu_do_idle();
	write_sysreg_s(saved, A9_CYC_OVRD);
	isb();
	return index;
}

static struct cpuidle_driver a9_idle_driver = {
	.name = "apple_a9_idle",
	.owner = THIS_MODULE,
	.states = {
		{
			.enter = a9_idle_wfi,
			.enter_s2idle = a9_idle_wfi,
			.exit_latency = 1,
			.target_residency = 1,
			.power_usage = UINT_MAX,
			.name = "WFI",
			.desc = "WFI (guarded boot clock gate)",
		},
		{
			.enter = a9_idle_clock_gate,
			/* No s2idle hook: the core selects baseline state0. */
			/* Conservative test policy, not measured hardware timings. */
			.exit_latency = 10,
			.target_residency = 1000,
			.power_usage = UINT_MAX - 1,
			.name = "WFI-CG",
			.desc = "Forced WFI clock gate",
		},
	},
	.safe_state_index = 0,
	.state_count = 1,
};

static void a9_idle_probe_cpu(void *unused)
{
	u64 aidr;

	if (!a9_is_twister()) {
		pr_warn("apple-a9-idle: cpu%u unexpected MIDR=%08x; no implementation register reads\n",
			smp_processor_id(), read_cpuid_id());
		return;
	}
	aidr = read_sysreg_s(SYS_AIDR_EL1);
	pr_info("apple-a9-idle: cpu%u MIDR=%08x CurrentEL=%#llx PFR0=%#llx AIDR=%#llx arch_retention=%u CPU_CFG=%#llx CYC_OVRD=%#llx\n",
		smp_processor_id(), read_cpuid_id(), read_sysreg(currentel),
		read_sysreg(id_aa64pfr0_el1), aidr, !!(aidr & A9_ARCH_RETENTION),
		read_sysreg_s(A9_CPU_CFG), read_sysreg_s(A9_CYC_OVRD));
}

static int __init a9_idle_init(void)
{
	struct device_node *cpu;
	unsigned int count = 0;
	int ret;

	if (!enable && !probe)
		return 0;
	if (!(of_machine_is_compatible("apple,s8000") ||
	      of_machine_is_compatible("apple,s8003")))
		return -ENODEV;
	for_each_of_cpu_node(cpu) {
		if (!of_device_is_compatible(cpu, "apple,twister")) {
			of_node_put(cpu);
			return -ENODEV;
		}
		count++;
	}
	if (!count)
		return -ENODEV;
	if (probe) {
		on_each_cpu(a9_idle_probe_cpu, NULL, 1);
		pr_info("apple-a9-idle: reset-based deep WFI unavailable; no Linux resume-vector handoff\n");
	}
	if (!enable)
		return 0;
	if (!a9_is_twister())
		return -ENODEV;
	if (clock_gate) {
		if (of_machine_is_compatible("apple,j71t")) {
			a9_idle_driver.state_count = 2;
			pr_warn("apple-a9-idle: experimental WFI-CG; no CPU retention\n");
		} else {
			pr_warn("apple-a9-idle: WFI-CG requires j71t; using WFI\n");
		}
	}
	ret = cpuidle_register(&a9_idle_driver, NULL);
	if (ret)
		pr_warn("apple-a9-idle: registration failed (%d); keeping existing idle driver\n",
			ret);
	return ret;
}
device_initcall(a9_idle_init);

MODULE_DESCRIPTION("Apple A9 opt-in WFI clock-gate experiment");
MODULE_LICENSE("GPL");
