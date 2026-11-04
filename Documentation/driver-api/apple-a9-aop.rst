.. SPDX-License-Identifier: GPL-2.0-only

===============================
J71t AOP bring-up (experimental)
===============================

CONFIG_APPLE_A9_AOP is a built-in, default-off, read-only observer. With
``apple_a9_aop.debug=1``, root may read ``/sys/kernel/debug/apple-a9-aop/status``.
The driver maps five four-byte resources and issues only readl() on them.
The MMU necessarily maps whole pages; no debugfs operation can address the
rest of those pages. FIFO, interrupt set/clear, reset/remap, and firmware
memory are never accessed. There are no write file operations or IRQs.

The diagnostic node deliberately has no power-domains property. An observer
API locks the existing PMGR domains in child-before-parent order, records
genpd and raw hardware state, and skips wrapper reads unless all five domains
are ON with ACTUAL=TARGET=15 and no reset, disable or parent-off bits. It does
not resume or hold any power domain. Hardware auto-PM remains untouched; the
software locks cannot exclude autonomous transitions, and the raw auto-enable
bit is included in the capture. Enabling the observer is an explicit hardware
diagnostic opt-in, not protection against an external bus fault.

This is a Linux-time snapshot, after the ordinary PMGR provider's probe.
The existing AOP always-on policy and warm-boot retention policy are unchanged.
It is not a pre-PMGR bootloader trace and does not prove firmware identity.

``apple_a9_aop.attach=1`` also enables the read-only ``preflight`` check:

* Running CPU: retain everything, refuse unknown host/memory ownership.
* Stopped CPU: request the main and external files listed below, report sizes,
  then refuse the unresolved
  reservation/remap/patchbay contract. No image is copied to AOP memory.

The exact filenames are ``apple/aop/ipad6faop-20H350.bin`` and
``apple/aop/ipad6faop_ext-20H350.bin``. request_firmware_direct() avoids a
userspace-helper wait. Sizes are inventory checks, not authenticity checks;
the host extractor must validate the pinned component and image SHA256s.
Mailbox fault bits 18/19 in either control register also stop preflight,
without acknowledging or clearing them.

The old 56-bit-payload/8-bit-endpoint codec, management hello12, bounded SPU
queue descriptors, version2 subpackets, announcement32, subtype4 property
payloads/replies, and the 22-byte motion decoder live in ``a9-aop-protocol.h``;
rings, AFK words, the firmware tag table and the calibration file format in
``a9-aop-afk.h``. Both headers are pure functions, compiled unchanged by the
host test harness. The observer has no call path to them; only the active
driver below uses them. Generic 96-bit ASC mailbox/RTKit clients are unchanged.

CONFIG_APPLE_A9_AOP_IIO builds the IIO consumer registered by the active
driver. Samples are invalidated on parse/transport failure and become stale
after max(1 second, five report intervals). AP receipt time is used for
freshness and buffer timestamps; no AOP timestamp is exported.

No part of this work writes syscfg, NVRAM or boot firmware storage.

Probe-time power-off observation
==================================

``apple_a9_aop.state_probe=1`` is an independent, default-off, read-only
probe-time snapshot. It requires neither debug=1 nor ring_scan/ring_pa and
can coexist with either ring diagnostic. It excludes the active driver
before any active-driver allocation or resource access, even with active=1.
For the isolated experiment use::

  apple_a9_aop.state_probe=1 apple_a9_aop.debug=0
  apple_a9_aop.active=0 apple_a9_aop.attach=0
  apple_a9_aop.ring_scan=0 apple_a9_aop.ring_iio=0

From iOS, slide to power off, wait 30 seconds, then enter DFU from that off
state and boot Linux without an intervening iOS boot. Keep the existing
non-AOP boot arguments. Omit stale ring_pa/ring_pa_size lists. There is no
device access or power-off action performed by the driver.

At observer probe, the existing five-domain PMGR API reads the label-checked
AOP domain (observe-domains[1]) and its dependencies under genpd locks. It
invokes the wrapper callback only when every software domain is ON, hardware
ACTUAL=TARGET=15, and reset/disable/parent-off are clear. Until the PMGR
providers exist, probe defers without wrapper reads. No power-domain attach,
resume, state write, mailbox FIFO, IRQ or cache operation occurs.

The callback reads exactly two allowlisted u32 words: +0xb88 and +0x4020.
wrapper_b88 is the raw +0xb88 word, not CPU status. The single log line
is also frozen in root-only debugfs at
``/sys/kernel/debug/apple-a9-aop/state_probe``. For example::

  aop state: pmgr target=0xf actual=0xf wrapper_b88=0x00020001 b88=1 4020=1 -> powered execution=unknown

The b88 and 4020 fields are bit 0; PMGR target/actual are the raw hardware
nibbles. +b88 mirrors inbox +4008 in the J71t captures, including occupancy;
neither bit0 value establishes CPU execution. When AOP ACTUAL=0, no wrapper
word is read: wrapper_b88/b88/4020 are marked skipped and the result is
unpowered. A transition, unavailable/error state, or another domain blocking
access reports unknown and guard_errno. Read failures do not fabricate PMGR
zero values. Previous running/halted classifications from b88 are withdrawn.

Read the saved line with::

  sudo cat /sys/kernel/debug/apple-a9-aop/state_probe

Debugfs reads issue no hardware accesses. With debug=0, only state_probe is
exposed and only those two wrapper resources are mapped. Explicit debug=1
retains the normal live status/preflight files when not in ring mode. Ring
mode alone still bypasses wrapper access; adding state_probe authorizes its
separate two-word snapshot. The probe runs after the ordinary PMGR provider,
whose existing always-on policy may already have powered a domain; it is not
a pre-Linux rail measurement. Hardware automatic gating is not inhibited.

Staged attach research (2026-10-25)
==================================

``apple_a9_aop.attach_stage=1`` freezes four status registers once at probe:
Raw wrapper +0xb88 (not CPU status), inbox control +0x4008, outbox control
+0x4020, and idle status +0x4040. The same five-domain PMGR guard precedes the reads. A domain
error skips the wrapper; mailbox faults or disabled outbox end
the observation. CPU execution remains unknown for every b88 value. An enabled
empty outbox is reported as observed-mailbox-enabled-empty. Pending data is
reported without consuming it. Read the
frozen result at ``/sys/kernel/debug/apple-a9-aop/attach_status`` (0400), or
in the kernel log. Repeated file reads issue no hardware transactions.

Use debug=0, state_probe=0, attach=0, active=0, ring_scan=0, ring_iio=0 and
omit ring_pa/ring_pa_size. Staged mode refuses combinations with any other
observer mode. Stage 1 excludes the active driver. Stage 2/3 instead arm it
with active=0 and debug=0, under the owner's accepted experimental crash risk.
Values outside 0..3 fail probe. All selectors are immutable boot parameters,
and 0 preserves existing behavior.

Stage 2 uses the bounded s2b ladder: passive 5 s; v12 HELLO_REPLY 2 s;
mailbox interrupt unmask 1 s; ping(seq0) 0.5 s; type6/0x220 2 s;
HELLO_REPLY 2 s; type0xb/0x20 1 s; HELLO_REPLY 2 s. A complete EPMAP
ends the ladder immediately. A real HELLO is answered even after a speculative
reply. Pre-HELLO power/ping ACKs are logged without claiming a handshake.
With the separate immutable ``apple_a9_aop.cycle=1`` opt-in (s2c), append
type6/0x201 0.5 s, type6/0x220 2 s and a final HELLO_REPLY 2 s. This requests
a management sleep/wake transition, not an AON reset. The default is off.
The 15.5 s / 20 s silent windows are followed on success by 2 s of replies.
It ACKs ordered EPMAP chunks, sends AP readiness=1, and stops. No endpoint is
started or buffer granted. Linux IRQs remain off: one poller consumes RX.
With wake=0 the only extra MMIO writes are masks 1/0x1000 to wrapper +4004 and +c04,
matching iOS's mailbox enable helpers. No CPU, auto-boot, remap or firmware
resource is used. There are no AON power/reset writes.
Stage 3 performs that same handshake then starts application endpoints,
answers AFK v8 requests from the dedicated 4 MiB no-map pool at 0x858000000,
and uses the existing calibration/10 Hz accel/IIO path. Missing calibration
is logged and the experiment can expose uncalibrated reports. The pool has
no reusable/shared-dma-pool flag and published slices are never freed.

Both stages bypass running-image/ext-lifetime proof deliberately. There is
one attempt per boot and no cold-loader fallback.
Read ``apple-a9-aop-active/{state,registers,startup_transcript,transcript,rings,samples}``
under debugfs. Startup mailbox words are retained separately from the rolling
trace; the startup budget is bounded and failure halts further polling/TX.
Samples contains the latest 128 raw accel reports with AP timestamps.
Registers retains up to 640 status snapshots before/after steps and TX,
including ordered immediate and 50 us TX readbacks. It never reads FIFO data
for a snapshot. All actual FIFO words are in the transcripts. CPU control,
raw wrapper +b88, mailbox set/clear and mirrors, both FIFO controls and idle are
logged; low status bits remain raw because their counter layout is unknown.
Debugfs only reads saved RAM. ``auto_boot_valid=0`` explicitly marks that the
experiment did not read auto-boot (the historical zero was a placeholder).

Hardware wake isolation (s2d..s2h / s3d)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Immutable ``apple_a9_aop.wake=1..5`` selects a separate sequence in stages 2/3.
It rejects cycle=1 and values outside 1..5 before mapping anything. After a
5 s passive window, perform exactly one hardware action and observe for 1 s:

* 1 (s2d/s3d): clear the remaining mask bits4/8 (0x110) at +4004 and +c04.
  IOP inbox-not-empty/outbox-space direction is a hypothesis from the four
  source layout and iOS's AP-side 0x1001 enables, not a hardware result.
* 2 (s2e): clear only bit4 (0x10) at +4004 and +c04, isolating inbox IRQ.
* 3 (s2f): +44 RMW bit4=1, preserving every other bit; no clear or reset.
* 4 (s2g): the same +44 RMW, then auto-boot at physical 0x210000600=1,
  matching iOS setCPURun(1). Only modes4/5 map that 4-byte resource.
* 5 (s2h): 0x110 to +4004/+c04, then +44 RMW bit4=1 and auto-boot=1.
  This isolates a combination if neither masks nor release alone works.

The superficially similar iOS b4e830 stores +c004=0x11 / +c024=1 belong to
the NMI path (RTBuddy _signalNmi), not ordinary mailbox wake. They are never
read or written here. No type15 NMI request is sent.

The remainder is ping0 (0.5 s), type6/0x220 (2 s), HELLO_REPLY (2 s),
type0xb/0x20 (1 s), HELLO_REPLY (2 s). Silence means 13.5 s of windows,
five TX words and at most four hardware stores. No sleep/reset/load fallback.
A complete EPMAP ends subsequent actions; a passive success avoids hardware
stores entirely. The same boot chains stage 3 if requested. Every hardware
store is logged before writing, read back, and bracketed by saved snapshots.
Snapshots additionally include a validity-tagged auto-boot value in modes4/5.
A fault or failed protocol stops the attempt; debugfs never retries.
These writes do not establish that unknown firmware is RTKit or that the CPU
is executing. The b88 bit0 execution interpretation is withdrawn; WFI and
internal IRQ masking remain unobserved.

The no-map AFK reservation can have a same-size BUSY generic ``reserved``
child created from the persistent FDT header reservation. Stage 3 claims an
exclusive resource underneath those exact generic nodes with
devm_request_resource(), preserving both reservations. It refuses System RAM,
other owners, partial overlaps and unexpected depth. It does not remove a
reservation or fall back to an ordinary allocation when the claim fails.

The iOS host code does not establish that duplicate HELLO_REPLY, AP power
notifications or endpoint stop/start release a surviving firmware's old DMA
addresses. A new EPMAP alone cannot prove that release. The older speculative
Round 4 attach below remains a separate experiment, is excluded by staged
mode, and is not recommended for the currently observed inherited instance.
The current experimental handoff is workspace docs/periph/aop-stage2b.md;
docs/periph/aop-stage23.md describes the earlier stage-2 device attempt.

Active driver (CONFIG_APPLE_A9_AOP_ACTIVE)
==========================================

A separate built-in driver binds ``apple,j71t-aop`` only with the immutable
boot parameter ``apple_a9_aop.active=1``; otherwise probe returns before any
mapping. It shares the ``apple_a9_aop.`` parameter prefix with the observer,
and with ``apple_a9_aop.debug=1`` adds root-only
``/sys/kernel/debug/apple-a9-aop-active/{state,transcript,rings,patchbay}``.

Linux RTKit is not reused: its mailbox carries the endpoint in a separate
32-bit word and its management type field (bits 59:52) overlaps this
transport's endpoint byte (bits 63:56). The driver keeps a small codec for
the A7IOPV2 64-bit FIFO (inbox ``0x4010``, outbox ``0x4038``), the RTBuddy
management protocol 12, and SPU/AFK v8.

Bring-up runs from a work item once the root filesystem provides
``/lib/firmware/apple/aop/``:

* All five PMGR domains must already be ON (checked with the observer API);
  a domain is never forced on. Mailbox fault bits stop everything and are
  never cleared.
* Running CPU with an empty outbox: the read-only fingerprint preflight and
  bounded ping proof described below; the one-shot HELLO_REPLY additionally
  requires ``apple_a9_aop.attach=1`` (default off).
  Running CPU with pending words: the first word must be the device HELLO,
  otherwise refused and the outbox is no longer read.
* Stopped CPU: the main image is copied to the AOP region (IOP link base
  ``0x01000000``), the tag table is patched, the external segment staged, the
  remap window and mailbox masks programmed, and the CPU released through CPU
  control bit 4 plus the auto-boot register.
* HELLO is answered with ``0x00200000000c000c``; EPMAP chunks are answered as
  iOS does; after the last chunk AP power and STARTEP for every application
  endpoint are sent, and a 1 s ping starts (5 s without an ack invalidates the
  stream; the AOP is not reset).
* Per endpoint the driver waits for the device's AFK ``0x80`` (version 8),
  answers ``0xa0``, allocates each ``0x89`` buffer with dma_alloc_coherent()
  (no IOMMU, 48-bit physical reply ``0xa1``), validates the AOP-initialised
  ``0x8a/0x8b/0x8c`` rings, sends ``0xa3`` and reads the IOP->AP ring on ``0x85``.
* The ``accel`` ready report (service id ``0x10000007``) binds the stream.
  The optional per-device ``apple/aop/accel-calibration.bin`` is sent in iOS
  table order, then batch/raw/report intervals 100000/0/100000 us, each
  waiting for its reply. The IIO device is registered after the first
  ``0xc2`` report.

IIO: ``in_accel_{x,y,z}_raw`` are AOP report counts, ``in_accel_scale`` is
0.000149638 (16.16 g in m/s^2), ``in_accel_sampling_frequency`` is the
acknowledged 10 Hz, ``in_accel_mount_matrix`` comes from the DT
``mount-matrix`` (identity until measured; ARot is applied by the firmware and
never again on the AP). A triggered buffer is fed by the device's own trigger
``apple-a9-aop-accel-devN`` once per report. Raw reads return -ENODATA before
the first report, after a transport failure, and when stale.

On shutdown/kexec the stream is stopped (intervals 0) and every endpoint gets
AFK shutdown ``0xa4``; the rings live in memory the next kernel reuses. If an
endpoint does not acknowledge, do not kexec; cold boot instead.

Running attach (Round 4)
=======================

The final opt-in command line is::

  apple_a9_aop.debug=1 apple_a9_aop.active=1 apple_a9_aop.attach=1

The observer owns the single immutable ``attach`` parameter and remains
read-only. ``attach=1`` alone does not enable the active driver. With
``active=1 attach=0``, Phases A/B run but no speculative HELLO_REPLY is sent.
The stopped-CPU loader and pending-device-HELLO path keep their existing
behavior. The running/empty path never writes CPU control, auto-boot, remap,
masks, firmware region or tags; it never resets, clears faults or forces power.

* Phase A (``attach-preflight``): snapshot includes CPU control and auto-boot.
  SHA-256-pin both reference firmware files, compare region head/UUID
  (0..0x60), build string (0x47440, 64 bytes), and the exact tag table shape
  (0x389b8, 0x1df bytes). Validate loader tags including EC0p, WrAd, McRA,
  nCal, SOC_ and SOCR. Compare all 0x90a0 ext bytes only after proving the
  covering pages belong to the AOP region or a plain non-reusable reserved
  memory carveout with no other driver/pool owner. Mapped RAM also requires
  PageReserved on every covering page; use its normal WB mapping, never a
  Device alias. No-map ranges use IO reads. There is no late reservation.
  Unreserved EC0p is refused before mapping it, even if its bytes might
  still match. All Phase A failures have zero mailbox traffic.
* Phase B (``attach-ping``): at most three pings, each with a 300 ms deadline,
  and two consecutive acknowledgments required. Accept seq or seq+1, also
  in the cold-path watchdog. Probe sequences 1, 3, 5 avoid overlapping ack
  windows. Any unexpected endpoint/type/sequence refuses immediately.
* Phase C (``attach-hello``), only with both gates: send exactly
  ``0x00200000000c000c`` once, wait at most 2 s for endpoint-zero EPMAP.
  Silence or anything else is ``refused-running-handshake-owned``. A loud
  zombie warning calls for a real power cycle, never a retry or kexec.
  Shutdown preserves the refusal and does not claim the AOP was stopped;
  the platform shutdown callback cannot veto a user-requested kexec.
* On EPMAP, use ordinary EPMAP replies, AP power, STARTEP >= 0x20, AFK init,
  GET_BUF/rings, ready report, calibration, 10 Hz rate, first report and IIO
  registration. Unlike the older cold path, attach requires a nonempty
  factory calibration file; absence or a failed command stops bring-up.

Debugfs ``state`` reports the phases, proof flags, attempt/ack counts and
whether the one HELLO_REPLY was sent and EPMAP accepted. ``transcript`` records
the phase for every mailbox word. ``patchbay`` includes the read-only running
head/build/tag snapshot. Failure strings distinguish foreign-image,
no-loader-tags, tag-mismatch, stale-ext, unreserved-ext, unexpected-mgmt,
no-ping-ack and handshake-owned.

Offline verification establishes parsing, bounds and policy only. The
firmware's reaction to the speculative reply and EPMAP proving first-AP
ownership remain inferences from the takeover design, not device-verified
behavior. No device measurement is supplied by this change.

Frozen fingerprint diagnostics (Round 5)
======================================

With debug=1, the read-only root-only file
``/sys/kernel/debug/apple-a9-aop-active/fingerprint`` explains Phase A even
after ``refused-running-foreign-image``. It reports:

* First 0x100 region bytes and their pinned-file counterparts, as hex/ASCII.
* The 0x40-byte UUID header at +0x20, with actual/expected words and decoded
  magic, version, table offset and table length.
* All 64 build-window bytes at +0x47440 as hex and bounded printable text,
  including embedded NULs. This window includes bytes after the build string;
  a matching text substring does not prove the entire window matches.
* All 0x1df tag-table bytes at +0x389b8, including raw little-endian words and
  the final three bytes even if the first TLV is malformed. Expected raw
  bytes are the UNPATCHED reference template; the expected loader values
  (including variable EC0p and entropy tags) are stated separately.
* A first mismatch and byte count for every window, reference-load/pinning
  status, and the saved wrapper remap configuration at +08/+10/+18/+20/
  +28/+30/+38 alongside the cold-loader expectation.

Every value comes from the original bounded Phase A snapshot. Reading the
file only formats RAM copies: no fresh MMIO, FIFO reads, mailbox TX, retries
or guessed physical mappings. Extra prefix/remap reads occur only with
debug=1, after the existing powered/running/empty/fault-free checks. Remap
values are evidence only and are never followed to a different address.

The acceptance rules, fixed offsets and one-shot attach policy are unchanged.
A different build string cannot authorize attachment. Support for another
image or a relocated runtime layout needs its own verified image/layout,
tag and ext-buffer contract plus the same reservation and handshake proofs.

Runtime identity and aperture width (Round 6)
===========================================

The iOS reader for this firmware is RTBuddy_memcpy_from32, not generic memcpy.
UUID, TLV and version-string reads use aligned 32-bit transactions, including
an aligned final word copied into local RAM for a short tail. The previous
arm64 memcpy_fromio path used 64-bit transactions and byte tail accesses.
The driver now uses ordered readl transactions in the same allowlisted region.
The declared 0x1df-byte table needs 0x1e0 bytes of bus coverage; the extra byte
must fit the mapped resource and is discarded in RAM. IO-mapped ext reads use
the same helper after the existing reservation proof. Normal mapped reserved
DRAM retains its WB mapping and ordinary memory reads.

The old 64-byte build window is still dumped, but is not all immutable version
text: the pinned string is 49 bytes plus NUL, then padding/runtime globals.
Firmware startup copies STKG to +0x47474, and +0x47478 is a mutable cursor.
Identity compares exactly the string including NUL; the guard must separately
match the four-byte STKG in the validated, exact-layout tag table. This avoids
classifying normal firmware initialization as a foreign image. Every unknown
identity, inconsistent loader/tag/guard, stale or unreserved ext, bad ping or
owned handshake still refuses with the existing distinct result strings.

The existing stopped-CPU loader also now uses aligned writel/readl copies, as
iOS does. Its stopped/empty/domain/fault gates are unchanged. Running attach
has no path to that writer and gains no reset, CPU control, auto-boot, remap,
mask, image/tag or power-domain writes.

The observed remap AP target remains 0x210e00000; the readback +28=0x1ff000
does not by itself locate another image. iOS writes descriptor length there,
but the hardware readback interpretation is unestablished. No larger resource
is mapped and no guessed DRAM or IOP address is followed. The firmware's
linked VA0x01000000 maps onto its PC-derived execution backing; it is not a
second AP physical address. Non-default split-data/header layouts remain
refused rather than following unvalidated pointers.

Correct-width identification must be measured on the device before declaring
its current image known. Attaching to an already-handshaked iOS instance is
still unproven and remains refused. AP power-off/AFK shutdown do not establish
a stopped unknown-owned core. The supported iOS provider stop writes CPU
control/auto-boot and changes power state, which this attach path may not do.
A real AON power cycle (or a separately validated clean stop by the original
owner) is the fallback; domains OFF still require separately designed
legitimate provider startup, not forcing a domain here.

Round 7: unsupported inherited layout and recovery boundary
=========================================================

The Round 6 device capture, with aligned readl32 transactions, still has no
pinned UUID/header or build text at the fixed offsets. The access-width fix
did not recover that inherited instance's image. Its live code physical
address and firmware identity remain unknown; no alternate layout is added.
Neither a build substring nor the 0x1ff000 remap readback authorizes a larger
AP mapping, a physical scan, or following an IOP virtual/physical pointer.

There is new static evidence in the pinned main firmware. Linked function
0x010349f0 translates a firmware VA through 0x0103062c/0x01027e3c (ARM
ATS1CPR and PAR), then invokes 0x0103028c. The latter interprets wrapper
+08/+10 as an additive delta, +18/+20 as a lower bound and +28/+30 as an
inclusive upper bound, comparing 32-bit physical page numbers after shifting
by 12 (a 44-bit address view).
For in-range IOP physical addresses it returns physical + delta; otherwise
it returns physical unchanged. This requires the actual code backing address,
which the current AP snapshot does not provide. It is not a decoder from
the firmware's linked VA to a newly authorized AP resource. The iOS writer
b4e72c stores descriptor length masked with ~0xfff at +28; the firmware's
inclusive software comparison does not establish hardware size/readback
semantics or justify changing the stopped loader to size-minus-one.

The inspected iOS normal load still copies unchanged OSData to fixed target
offset zero (b68cdc/b69000, b6f340); the pre-copy SPU callback b610f8 is RET.
The preloaded split-segment branch requires pre-loaded, and extra SRAM access
setup requires nonzero sram-index. Both properties are absent on the saved
j71t nub. rtb_force_dram defaults false. Those alternative input paths do
not establish a relocated, decompressed or data-first runtime image.

Phase A now samples all seven remap words around the existing allowlisted
head/build/tag reads, including when debug is disabled. Both samples and
the AP resource must match the single supported contiguous cold-loader
descriptor: region 0x210e00000/0x9b000; words 10e00000,2,0,0,9b000,0,1.
This is a conservative software whitelist, not device verification of cold
readback. A changed/unsupported descriptor still yields
refused-running-foreign-image before ext mapping, ping or HELLO_REPLY.
All existing tag/guard/ext/reservation/ping/ownership refusals remain.
The stopped cold path is unchanged. No power/reset/CPU-control/auto-boot/
remap/mask/image/tag writes are added to inherited-running attach.

Frozen state/fingerprint output reports alternate_layouts=0, both remap
samples, stability, whitelist match, and the firmware's translation view.
It performs no fresh MMIO; snapshot_end=1 terminates every fingerprint
report so a collector can detect truncation. The retained Round 6 combined
capture cuts off within its prefix and has no raw table bytes; its complete
patchbay header/build and printed mismatch counter still establish refusal,
but cannot identify an unseen structure or another build.

Original-owner identification would need the live IOService FirmwareUUID,
RTKitVersion and FirmwareVersion properties (b6f6dc), its target descriptor
PA/length/segments and the corresponding main/ext bundle. The saved
DeviceTree-plane dump lacks that map. A matching version alone cannot prove
Linux ownership/reservation or a stateless handshake.

AOP-only cold-recovery design, not an implemented power-cycle path
----------------------------------------------------------------

The vendor's stop/start is not proof of an AOP-only PMGR cycle. Actual V2
vtable 6d3ff98 has idle slot +6d8 -> b4c460 returning constant success,
+6a8 -> b4e7c4 RET, and +6c8 -> b4b6f4 controlling an IOMapper, not PMGR.
setCPURun b4e7c8 changes +44 bit4 and auto-boot. Generic powerOn/powerOff
calls gate index zero, but this board's saved AOP clock-gates/power-gates
arrays are empty and aot-power=1. AppleARMIODevice b9bc178 sets gate counts;
its b9bbdac/b9bbe58 methods reject index zero for empty arrays. Thus this
loader path supplies neither AOP isolation proof nor a PMGR order/dwell.

Linux topology is gpio -> aop -> busif/cpu/filter. GPIO pinctrl consumes
aop_gpio. aop is always-on; integration warm keep_on independently refuses
power-off. Global kexec retention must remain enabled for unrelated devices.
PMGR ACTUAL=0 alone does not establish DMA drain, mailbox/SRAM state loss or
CPU restart defaults. The GPU recovery exception is not AOP evidence.

A separate recovery implementation would require the following contract:

* Distinct immutable recovery opt-in, exclusive with attach; pin board,
  AOP-only topology, firmware hashes and AP resources before mutation.
* Serialize genpd and every affected GPIO/IRQ consumer; quiesce them and
  establish an AOP-specific master isolation/drain proof. A valid original
  owner may cleanly stop; a lost owner requires power-cycle recovery, not
  a claimed protocol shutdown. AP power-off or AFK shutdown is insufficient.
* Through a provider-owned transaction, release only affected AOP holds;
  request power-gate zero child before parent, check target/actual and
  reset/minimum constraints with bounded waits. Parent ordering and off dwell
  need AOP-specific evidence. Never read AOP MMIO with dependencies off.
* Restore parents before children with AUTO disabled; require stable active
  dependencies, CPU-running zero, empty fault-free FIFOs and understood
  retained auto-boot/remap state before any image write. Abort if old code
  restarts. Never clear a fault to make progress.
* Allocate fresh Linux-owned ext; stage pinned ext and patch main in RAM.
  Verify max(image,RTSZ) capacity; aligned u32 copy/readback; vendor remap
  order +18/+20,+28,+30,+08/+10,+38. Resolve and verify remap readback before
  CPU release. Apply known masks/outbox setup; CPU +44 bit4 then auto-boot=1;
  existing bounded HELLO/EPMAP/AP-power/AFK/rings/calibration/10 Hz/IIO setup.
* One bounded attempt per transition; no power/reset/HELLO retry. Partial
  failure retains domain evidence and potentially live DMA buffers. Blind
  repower rollback or freeing buffers without a stopped-master proof is unsafe.

Unresolved: true main mapping, isolation/reset scope, parent ordering/dwell,
GPIO/IRQ retention and all consumers, watchdog/mailbox/SRAM/remap state loss,
DMA lifetime and partial-failure containment. No safe runnable AOP-only cycle
is established, so no recovery parameter or power mutation is provided.
Warm kexec can collect these diagnostics without DFU; neither operation is
an AON power cycle. A real rail cycle still requires a measured stopped
baseline and separately validated legitimate domain startup.

Inherited ring reader: E1 discovery and E2 read-only IIO
=====================================================

This is an independent, default-off experiment for the iOS-left AOP session.
Use ``apple_a9_aop.debug=1 apple_a9_aop.ring_scan=1`` for E1, or
``apple_a9_aop.debug=1 apple_a9_aop.ring_iio=1`` for E2. Set active=0 and
attach=0. Any ring mode excludes the active driver's probe before
resource mapping, IRQ requests or power/DMA operations, even if active=1 is
accidentally present. Ring mode bypasses the observer's wrapper maps and power
provider lookups too, unless state_probe separately requests its guarded
two-word snapshot. Its ring debugfs files are ring_scan, ring_reader and/or ring_pa;
status/preflight are absent. With all ring gates zero, existing behavior is
unchanged. Probe does not start a scan. Ordinary E1 starts on the first
debugfs read; optional early capture is described below.

Directed capture from an iOS-probed AFK buffer PA is also available using
``ring_pa`` and ``ring_pa_size``. It automatically captures at the same
postcore phase, before any broad scan, even with ring_scan=0. See
:doc:`apple-a9-aop-target` for the bounded lists, RAM policy and frozen output.

E1: reading the root-only ``/sys/kernel/debug/apple-a9-aop/ring_scan`` performs
one scan per boot and freezes the result. Subsequent reads and seq_file buffer
retries only format the saved result. It inventories full pages in memblock
RAM, including no-map ranges for coverage, but only reads pages proven to be
System RAM by the resource tree. By default it skips memblock reservations
and PageReserved pages; no-map and invalid linear-map pages are always skipped.
Readable pages require
pfn_valid, pfn_is_map_memory and virt_addr_valid before page-confined
copy_from_kernel_nofault through phys_to_virt. No /dev/mem, guessed MMIO,
ioremap, page reference, allocator lock or cache clean/invalidate is used.
Pages covering the reader's allocations and the reading task's kernel stack
are excluded, so copied headers cannot be rediscovered as false live rings.
Memory hot removal is excluded during the scan/watch. Synchronous copy faults
quarantine only the faulting PFN, record it and continue with the next page.
This also applies to cross-page ring copies and watch-time faults: discard
the unreadable hit, continue observing the others, never retry a skipped PFN.
E2 retains its separate fault latch. Fault-contained RAM copies do not promise
recovery from an asynchronous hardware SError.

The limits are 2 GiB of page traversal, 45 seconds of scan time, 16 hits, and at
most 256 KiB per 4 ms (about 64 MiB/s, plus processing time). A fatal signal
aborts. Every candidate is 64-byte aligned, has exactly bufsize=0x3f40 and
marker=0x00070006, valid aligned rptr/wptr and all 16 KiB inside the same full-page
memblock RAM span, with eligible System RAM covering pages. Found rings are watched together
for two seconds at 100 ms intervals. The output includes PA, header/pointers,
observed pointer changes, first 0x100 bytes, bounded EPIC parse status, report
sequence/timestamp/axes and current page state. A changing pointer is evidence
of writes to a candidate, not by itself proof of an accel RX ring.
The discovery wptr is separate from the first watch wptr: changes before the
two-second observation window are not counted as advancement within it.
Missed pointer wraps, dead streams, overwritten headers or noncoherent DMA can produce
false negatives. Bounds/deadline/hit-limit results are incomplete searches.

Format 2 reports readable bytes, skipped bytes, total full-page RAM,
unvisited bytes, traversed bytes, start boottime and capture phase. Readable
and skipped page sets are disjoint: a later watch fault revokes an earlier
successful read. Per-reason totals distinguish no-map, reserved, non-System
RAM, invalid linear map, reader/stack exclusions and read faults. The report
stores at most 256 reason-tagged ranges (exclusive ends), coalesces consecutive
same-reason pages, and explicitly counts omitted bytes if that storage fills.
Coverage counters and PFN quarantine remain exact even then. Tracking is
bounded to a 4 GiB DRAM address span; a larger span refuses allocation.

Optional ``apple_a9_aop.ring_scan_reserved=1`` requires debug=1 and ring_scan=1.
It also admits reserved linear-mapped DRAM, including crash-kernel, CMA and
other memblock/PageReserved reservations, after the same full-page memblock
RAM, System RAM resource, pfn_valid, pfn_is_map_memory and virt_addr_valid
checks. No-map, MMIO, reader/stack exclusions and fault quarantine remain
unconditional. No mapping, reservation or permission is changed. In particular,
the AOP firmware window at 0x210e00000 is not DRAM and is never added to E1.
This modifier works with either lazy or postcore capture, and its effective
value is frozen into ``ring_scan_reserved`` in the report. Coverage counters
and skipped-reason/range reporting retain their existing meanings.

Every hit reports its PA and ``range_class``: system-ram, crash-kernel, cma,
reserved-dram or mixed. Crash-kernel membership uses IORES_DESC_CRASH_KERNEL;
CMA membership uses the kernel CMA-area iterator and cma_intersects, including
discontiguous areas. Other reserved membership uses memblock/PageReserved.
These are metadata labels, not guessed address windows. A ring covering
different page classes is mixed. Labels are captured at discovery and do not
establish that Linux never wrote the bytes: CMA can have consumers and the
kernel/boot-payload reservation has already been overwritten. An advancing
pointer and valid 0xc2 payload remain required to identify a live accel ring.
E2's separate plain no-map reservation contract is unchanged; a hit in a
crash-kernel or CMA region does not authorize its automatic reuse by E2.

Optional ``apple_a9_aop.ring_scan_early=1`` requires debug=1 and ring_scan=1.
It runs the same one-shot scan/watch at postcore_initcall_sync, after memory
management/scheduling and the arm64 RAM resource map exist, before platform
device population and most device-driver allocations. The early result is
retained and adopted by the observer; debugfs never replaces it with a late
scan, even on early failure. The phase is ``postcore-initcall`` (or
``early-unavailable`` on failure); the default is ``debugfs``. An early scan
can delay boot by up to about 47 seconds. It still occurs after Linux's initial
allocations, and on a warm kexec the previous kernel already reused RAM.
For the best cold-boot evidence, put debug=1, ring_scan=1, ring_scan_early=1
and, when requested, ring_scan_reserved=1 in the FIRST Linux boot
after iOS -> DFU -> Linux, without an intervening Linux or AON power cycle.

A late debugfs scan cannot establish that pages were never allocated. Output
explicitly reports historical_allocation=unknown. PageReserved, nonzero page
reference counts and PageBuddy heads are lockless snapshots, not ownership
proofs. Free pages in the buddy/PCP allocator are still Linux-owned and may
have held data earlier. All nonreserved covering pages are counted as
allocator_pages. Candidate hits in these pages produce a critical warning;
advancing hits produce a second LIVE warning about latent DMA corruption.
Nothing pins, claims, reserves, repairs or writes those pages on this boot.
Linux allocations for the scanner itself may already have destroyed evidence.

E2 requires an orchestrator-supplied ``memory-region`` phandle on the observer
node and optional ``apple,ring-offset``. The phandle must point directly under
/reserved-memory to one static, available, plain no-map region with no
compatible, reserved-memory ops, reusable flag, overlapping sibling region or
other memory-region consumer. Its page-aligned range must cover exactly the
pages containing the 16 KiB ring, starting at the page containing the ring
header; offset must be below PAGE_SIZE and 64-byte aligned. The runtime DT
range must match the early reserved_mem entry. Every byte must be memblock
RAM and every covering page must be excluded from the linear map. The standard
early OF reserved-memory path makes the exclusion before allocation. This
driver never calls memblock_reserve or accepts a late overlay as a substitute.

Only after that proof does arm64 __ioremap_prot with PAGE_KERNEL_RO create a
read-only, normal WB mapping, with no writable linear alias. Device mappings
of ordinary RAM would violate arm64 alias rules and are not used. Mapping a
no-map carveout does not establish AOP/AP cache coherency; advancing, plausible
reports remain a device gate. No cache maintenance is attempted on inherited
buffers. The same exact header is revalidated at this PA. The private cursor
starts at the current wptr, ignoring historical samples; within two seconds a
new wptr and valid motion report must arrive before IIO registration. Both
known entry magics (AOP and IOP) are accepted; their direction is unproven.
Report category and subtype 0xc2 identify the accel payload, never the magic
alone. Notify/reply subpackets cannot supply samples.

When IIO is built in, ``apple-a9-aop-ring-accel`` provides three read-only raw
axes, shared scale 0.000149638 m/s^2 per AOP report count, and mount matrix.
The MPU-6900 firmware reports are interpreted as signed 16.16 g, not 16-bit
MPU register counts. The scale and residual screen orientation require the
six-pose device check. Firmware calibration/ARot is retained, never resent or
applied twice. Without a mount-matrix property the IIO helper uses identity;
this is provisional until measured. There is no rate control, advertised
sampling frequency or buffer. With IIO disabled or modular, E1 still builds
and ring_iio refuses with EOPNOTSUPP.

Polling runs every 20 ms using a private snapshot bracketed by stable published
pointers and read barriers. Only a changed publication with a new report
sequence/timestamp refreshes freshness. Raw reads return ENODATA after 500 ms
without a fresh report or wptr, on malformed data or faults. A fault/protocol
failure latches, cancels polling and does not retry. Debugfs ring_reader gives
cached PA, validated header, pointers, fresh flag, sample count and error.
Removal drains debugfs, unregisters IIO, cancels work and unmaps in that order.
Polling only touches private reader state, never the unregistered IIO device.

Neither path writes shared rptr, ring data, mailbox, CPU control, clock, reset,
power or sensor registers. Consequently the original producer may fill the
ring and stop permanently: a read-only observer cannot provide indefinite
streaming against a producer that requires rptr acknowledgment. Ring reuse
can fail even if iOS once delivered samples. A static/no-hit result is not
proof that AOP DMA has stopped elsewhere; other endpoints and buffers remain
unreserved. Reserving one accel ring does not contain all inherited AOP DMA.

The first measurement must follow an iOS boot with accel enabled, then DFU ->
palera1n -> pongoOS -> m1n1 -> Linux without an AON power cycle. An ordinary
Linux warm reboot cannot recreate the lost iOS session. Rediscover the PA after
any iOS boot. For E2, put the hint in the incoming boot DT before Linux starts;
never kexec from an unreserved E1 kernel on the assumption that a new DT can
protect pages the current allocator or kexec loader already owns. First boot
E2 through DFU while retaining the same iOS/AOP session, then test warm kexec
between kernels whose running and incoming DTs both retain the reservation.
Do not boot through iOS or cycle AON between E1 and this E2 attempt. If the
session is lost, redo E1; no new PA is selected automatically.

Forced cold restart experiment
------------------------------

``aop_force_cold=1 apple_a9_aop.attach_stage=2`` selects one forced stop and
cold HELLO/EPMAP boot; stage 3 chains the existing reserved-pool AFK/accel path
in the same boot. All other AOP modes, wake and cycle must be off. The selector
is immutable, default off, J71t only, and shares the atomic one-attempt guard.

Stop follows AppleA7IOP::stop (20H350 j71t ``fffffff005b4b8fc``): outbox enable
bit0 clear, mailbox masks set to 0x1111 at +4000/+c00, RUN bit4 clear at +44,
then AON auto-boot (0x210000600) = 0. The V2 reset/unmap hook +6a8 is RET at
b4e7c4; no separate core reset line is identified. Its idle hook b4c460 returns
success without testing hardware. V2 vtable 6d3ff98+680 leads to b4e4b8,
which reads +b88 bit0 through direct accessor b4e9b4 (+640). This word mirrors
inbox +4008 on J71t and cannot independently identify CPU run/halt. No reliable
halt status bit was found. Linux uses the owner-authorized fallback: after
clearing RUN and auto-boot, wait a fixed 10 ms, then require RUN, auto-boot
and outbox enable zero and fault-free mailboxes. Log predicate=run-clear-delay,
rawb88 and execution=unknown. The accepted command readback is not PC halt
or DMA-drain proof. Do not poll b88 for a transition or require its bit0 clear.
No power-domain cycle, NMI or invented reset bit is used.

After this gate only, discard at most 64 words per FIFO. Outbox pop +4038 is
verified in the iOS accessor b4e498. Inbox pop +4018 is an experimental AP-side
use of the receive half of the generic FIFO layout in the pinned firmware
0102ce1c/0102cdd8 (status+8, send+10, receive+18). There is no verified iOS AP
inbox-drain call. If either FIFO stays nonempty or faults, stop before loading.
Do not clear the fault bits. Stale words are logged, never dispatched.

The existing cold loader checks the pinned SHA256 of both files before using
them. It patches the normal tag table, stages ext at 0x858000000 in the retained
4 MiB no-map pool (16 KiB aligned), and copies/read-verifies the main image in
the exact 0x210e00000/0x9b000 no-map reservation. Recheck PMGR,
RUN/auto-boot/outbox-enable zero and empty/fault-free FIFOs before copying.
Recheck the stop controls before each stale FIFO pop and at each FIFO end.
A b88 bit0 that stays set through drain and firmware lookup is permitted.
Program remap in iOS
b4e72c order +18,+20,+28,+30,+08,+10,+38, then masks as b4b738, outbox enable,
RUN bit4 and auto-boot=1 as b4e7c8. Every control store logs its old value,
written value and ordered readback, with frozen surrounding register snapshots.
Image RAM is copied in aligned words and read-verified, not logged per word.

Only a device-originated HELLO starts the normal protocol; no speculative
HELLO or attach ladder runs. Forced HELLO polls synchronously for at most
20 s from release (the iOS boot-state timeout). After 1 s without HELLO,
send exactly one management type6/0x220 wake; re-drain before sending in case
HELLO arrived during SRAM sampling. No wake is sent after HELLO or a fault.
This is a retained-sleep hypothesis test: iOS b65cdc/b71fc4 sends this word
only for old status 3; forced Linux has no verified equivalent status.
The word is not a reset or a speculative HELLO. TX failure is terminal.
EPMAP waits 2 s, IOP readiness 2 s.
Firmware lookup is bounded at 180 s. Stage 2 stops before STARTEP/AFK traffic.
Stage 3 uses the remainder of the retained pool, calibrates/enables accel and
registers IIO only after the first valid report. No Linux RX IRQ is enabled
for either forced entry. Existing startup-word and pool bounds apply. Debugfs
state, registers, startup_transcript, transcript and (stage 3) samples/rings
provide RAM-only evidence; failures do not schedule another boot attempt.

Before release, freeze all 0x9b000 bytes in the reserved SRAM aperture.
Capture again at release, 10 ms, 100 ms, before/after the single wake,
5 s, and HELLO/deadline: at most eight frames. All SRAM accesses use aligned
readl under the existing PMGR guard. A failed guard prevents the callback.
No address is derived from SRAM contents. The two 620 KiB RAM buffers are
private diagnostic copies, never published to the AOP. Samples are non-atomic.
The frozen ``cold_memory`` file reports hashes, changed-word counts/first
deltas in boot/text-tags/data/BSS/scratch, runtime guard vs patched STKG,
remap/control/mask words and FIFO status at +b88/+ba0/+4008/+4020.
``cold_before`` and ``cold_after`` are binary RAM-only snapshots; the latter
is the latest successful sample. They remain readable after failure without
any hardware access. This retains possible log/crash bytes without claiming
an identified RTKit crash-buffer format. Unchanged SRAM does not prove the
CPU is stopped; changes alone do not identify their writer. A newly matching
STKG runtime guard plus expected page-table/BSS changes supports fresh entry.

The remap remains the iOS descriptor size 0x9b000, not RTSZ or RTSZ+ext.
+08/+10 are the literal PA halves 0x10e00000/2. Ext remains separately
mapped by firmware from the full EC0p PA; no fixed AP address, DART/SART
programming, new reset pulse or reset-vector store has been established.
Ext contents now receive an AP readback check before release as well.

Cold cache/IRQ isolation (owner-accepted crash risk)
---------------------------------------------------

``aop_cold_test`` is immutable and requires ``aop_force_cold=1``.
Zero preserves the stock cold experiment. Bit0 replaces the pinned reset
stub ARM instruction at +0x29c (MOVW r1,#0x1005) with MOVW r1,#0x1001,
leaving AOP D-cache disabled after the early C-bit clear. Both original
firmware hashes and the exact opcode must match before copying. The source
firmware files remain pinned. Bit1 clears IOP notification bits4/8 in both
+0x4004/+0xc04 after the vendor masks and before RUN. It adds no FIFO pop,
AP protocol word, domain transition or reset. Three tests isolate cache
visibility, IOP notifications and their combination; none is a verified fix.

The cold2 v3 backing SRAM includes patched STKG in a saved stack frame and
an ext-image return address while the runtime guard remains the image
initial value. AOP cached data is therefore a strong explanation for stale
AP-visible globals. Stable backing hashes are not proof of a stopped CPU,
and saved stack words are not a captured live PC or complete backtrace.
The same eight guarded SRAM/status frames and 20-second one-wake limit
apply to all variants. Stage2 must be reviewed before its matching stage3.

Cache-off v4 exposes a fatal before the first task switch: PRNG dispatch at
linked 01034e9c imports the seeded PRNG bytes, ECID and NONC, but ECID is zero.
01034fa0 branches to the panic at 0103501a; the recorded format is
``PRNG !patchbay ecid=%llx di=%p``. Stop code 4 at 0106ca3c leads to the
non-returning DSB/WFI loop at 010328d8/0102760c with IRQs disabled. The ready
worker has not run and no normal scheduler wait object can resume startup.

Bit2 (value 4) requires ``apple/aop/boot-ecid.bin``, exactly eight nonzero
little-endian bytes exported from this device's saved /chosen unique-chip-id.
The ID is never embedded in kernel source, a public DT or a command line.
The file is staged only in the private initramfs. Bit3 (value 8, requires
bit2) also writes eight fresh random NONC bytes; it never reuses boot-nonce
or touches NVRAM. Tests 5 (cache off + real ECID, NONC still zero) and 13
(cache off + real ECID + fresh NONC) isolate the proven missing prerequisite
and match the iOS pair respectively. Invalid combinations, missing/zero/
wrong-sized IDs or malformed tags fail before any firmware copy or release.
Both requested identity fields are validated before either is patched.
All prior snapshot, readback, single-wake and stage gates remain in force.

Deferred cold release and post-PRNG fence
----------------------------------------

``aop_defer=1`` requires forced cold and removes the probe-time work submission.
Probe only prepares resources and diagnostics. Root may write exactly 1 to
``/sys/bus/platform/devices/210800000.aop/cold_release`` after USB/SSH is ready.
Reading this file or debugfs never starts the experiment. Its ``triggered`` bit
is protected by the shutdown mutex; duplicate/concurrent writes and writes
after shutdown return EBUSY. A missing enabled ``ramoops`` console returns
ENODEV without consuming the trigger. No EFI pstore fallback is used.

The triggered worker retains the global one-attempt gate. Before stop, before
RUN, and after RUN but before auto-boot, it writes a console checkpoint, waits
at most 1 s for console output and drains stores. A missing RAM console or
flush timeout stops the attempt. A final checkpoint follows auto-boot.
The RAM console records continuously; no synthetic panic/oops is needed.
Both AP mailbox notification bits remain masked in both wrapper views;
stage 2/3 still never requests/enables Linux IRQs and polls the FIFOs.
IOP notification selection, reservations and the eight SRAM frames are retained.

``aop_defer=2`` also requires stage 2, ECID and D-cache off. It changes only
ext +0x4204 (linked 01072204) from Thumb MOVS r1,#0x14 to B . (e7fe).
The two original hashes, opcode, size, ext copy and patched word readback are
checked. Original firmware files stay unchanged. Execution reaches this fence
only after PRNG returns and 01072202 writes 0x21 to 01052ab8+0xc. It stops
before control-object 01047598 method 20 sets bit2 at physical 210008004 and
before task creation/IRQ enabling. The fence is an intentional permanent park,
not sensor firmware and not resumable. A 20 s no-HELLO result is expected.
Compare fence vs full release with the same identity. A fence failure cannot
be blamed on that subsequent control write. Neither survival nor cause has
been verified on hardware.

The tri49a baseline already has built-in PSTORE_RAM/CONSOLE, a 1 MiB console
in a 2 MiB no-map reservation, early ramoops registration, and inherited FDT
reservation retention through kexec. Keep that same running FDT during warm
recovery. Forced reset/DFU may erase DRAM or change the chosen physical region;
no RAM-only change guarantees recovery across that boundary. Collect the live
SSH console on the Mac before triggering and preserve recovery pstore archives.
