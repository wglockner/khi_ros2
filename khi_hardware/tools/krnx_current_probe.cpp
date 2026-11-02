/* KRNX joint current probe (not built by CMake; hand-compile like krnx_probe.cpp):
 *   cd khi_hardware && g++ -O0 -o /tmp/probe_current tools/krnx_current_probe.cpp -Iinclude -Llib/x86_64 -lkrnx -Wl,-rpath,$PWD/lib/x86_64
 *
 *   probe_current <ip> [seconds] [cont_no] [kind_hex] [zpow]
 *
 * READ-ONLY by default: sets the RT cyclic data kind BEFORE krnx_Open (manual 6.1), reports which kinds the
 * AS supports (as_kind), samples TKrnxCurMotionDataEx (joint current [A], saturation) and prints a summary.
 * 2026-10-28 on the CX110L E-controller: as_kind 0x1fff (no EXTRA_DATA, so no F/T stream option), current
 * is exactly 0 with the brakes applied, also with the motors ON and the arm idle; a nonzero reading needs
 * the servo loop holding (driver active, RTC on): JT3 10.7 A, JT2 -3.8 A, JT5 -1.1 A at home.
 * "zpow" ENERGIZES the motors through the monitor channel and powers them off after the sample:
 * arm idle, nobody in the cell, driver down. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "khi_hardware/krnx.h"

static int mon(int c, const char * cmd)
{
  char buf[16384]; int as = 0; memset(buf, 0, sizeof buf);
  int rc = krnx_ExecMon(c, cmd, buf, sizeof buf, &as);
  printf("---- ExecMon \"%s\" rc=%d (-0x%X) as_err=%d ----\n%s\n", cmd, rc, -rc, as, buf);
  return rc;
}

int main(int argc, char ** argv)
{
  /* 5th arg "zpow": ENERGIZE the motors (ZPOW ON via the monitor channel, the driver's own activate
   * command), sample, then ZPOW OFF. Arm idle, nobody in the cell, driver down. */
  const int do_zpow = argc > 5 && strcmp(argv[5], "zpow") == 0;
  const char * ip = argc > 1 ? argv[1] : "192.168.1.24";
  const double seconds = argc > 2 ? atof(argv[2]) : 3.0;
  const int cont_no = argc > 3 ? atoi(argv[3]) : 0;
  char host[64]; strncpy(host, ip, 63); host[63] = 0;
  /* 4th arg: cyclic data kind bits in hex; 0 = leave the controller defaults (angle, angle_ref, current, error) */
  unsigned short want = argc > 4 ? (unsigned short)strtoul(argv[4], NULL, 16)
                                 : (unsigned short)(KRNX_CYC_KIND_ANGLE | KRNX_CYC_KIND_ANGLE_REF | KRNX_CYC_KIND_ERROR |
                                                    KRNX_CYC_KIND_CURRENT_SAT | KRNX_CYC_KIND_ANGLE_VEL |
                                                    KRNX_CYC_KIND_ROBOT_STATUS | KRNX_CYC_KIND_CURRENT);
  if (want)
  {
    int rc0 = krnx_SetRtCyclicDataKind(cont_no, want);
    printf("SetRtCyclicDataKind(0x%04x) before open rc=%d\n", want, rc0);
  }
  else printf("leaving the default cyclic data kinds\n");
  TKrnxOpenErrorInfo ei; memset(&ei, 0, sizeof ei);
  int c = krnx_OpenSingleProcess(cont_no, host, &ei);
  printf("open rc=%d (err_no=%d port=%d pid=%d src=%s)\n", c, ei.err_no, ei.port, ei.pid, ei.source);
  if (c < 0) return 1;
  krnx_SetAuxApiTimeoutPeriod(c, 3000);

  TKrnxPanelInfo p; memset(&p, 0, sizeof p);
  krnx_GetPanelInfo(c, 0, &p);
  printf("panel: motor=%d error=%d teach_lock=%d emergency=%d repeat=%d\n", p.motor_lamp, p.error_lamp,
         p.teach_lock_lamp, p.emergency, p.repeat_lamp);

  unsigned short krnx_kind = 0, as_kind = 0;
  int rc = krnx_GetRtCyclicDataKind(c, &krnx_kind, &as_kind);
  printf("GetRtCyclicDataKind rc=%d krnx_kind=0x%04x as_kind=0x%04x  (CURRENT 0x0004 supported by AS: %s, "
         "CURRENT_REF 0x0020: %s, CURRENT_SAT 0x0040: %s, EXTRA_DATA 0x2000: %s)\n", rc, krnx_kind, as_kind,
         (as_kind & KRNX_CYC_KIND_CURRENT) ? "yes" : "no", (as_kind & KRNX_CYC_KIND_CURRENT_REF) ? "yes" : "no",
         (as_kind & KRNX_CYC_KIND_CURRENT_SAT) ? "yes" : "no", (as_kind & KRNX_CYC_KIND_EXTRA_DATA) ? "yes" : "no");


  if (do_zpow)
  {
    if (p.teach_lock_lamp || p.emergency || p.error_lamp) { printf("refusing ZPOW ON: teach lock / emergency / error lamp set\n"); krnx_Close(c); return 2; }
    printf("\n*** ENERGIZING MOTORS: ZPOW ON ***\n");
    mon(c, "ZPOW ON");
    int lit = 0;
    for (int i = 0; i < 100 && !lit; ++i) { memset(&p, 0, sizeof p); krnx_GetPanelInfo(c, 0, &p); if (p.motor_lamp) lit = 1; else usleep(100000); }
    printf("motor lamp %s\n", lit ? "ON" : "still OFF after 10 s");
    if (!lit) { mon(c, "ZPOW OFF"); krnx_Close(c); return 3; }
    usleep(500000);
  }
  /* sample */
  const int n = (int)(seconds * 100);
  float cmin[6], cmax[6], csum[6], smax[6];
  for (int j = 0; j < 6; ++j) { cmin[j] = 1e9f; cmax[j] = -1e9f; csum[j] = 0; smax[j] = 0; }
  int ok = 0, changed = 0;
  TKrnxCurMotionDataEx prev; memset(&prev, 0, sizeof prev);
  for (int i = 0; i < n; ++i)
  {
    TKrnxCurMotionDataEx md; memset(&md, 0, sizeof md);
    rc = krnx_GetCurMotionDataEx(c, 0, &md);
    if (rc == KRNX_NOERROR)
    {
      ++ok;
      if (i && memcmp(md.cur, prev.cur, sizeof md.cur)) ++changed;
      prev = md;
      for (int j = 0; j < 6; ++j)
      {
        if (md.cur[j] < cmin[j]) cmin[j] = md.cur[j];
        if (md.cur[j] > cmax[j]) cmax[j] = md.cur[j];
        csum[j] += md.cur[j];
        if (fabsf(md.cur_sat[j]) > smax[j]) smax[j] = fabsf(md.cur_sat[j]);
      }
      if (i % 100 == 0)
      {
        printf("t=%4.1fs ang[deg] %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f | cur[A] %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f | "
               "cur_ref %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f\n", i / 100.0,
               md.ang[0] * 180 / M_PI, md.ang[1] * 180 / M_PI, md.ang[2] * 180 / M_PI, md.ang[3] * 180 / M_PI,
               md.ang[4] * 180 / M_PI, md.ang[5] * 180 / M_PI, md.cur[0], md.cur[1], md.cur[2], md.cur[3], md.cur[4],
               md.cur[5], md.cur_ref[0], md.cur_ref[1], md.cur_ref[2], md.cur_ref[3], md.cur_ref[4], md.cur_ref[5]);
      }
    }
    else if (i % 100 == 0) printf("GetCurMotionDataEx rc=%d (-0x%X)\n", rc, -rc);
    usleep(10000);
  }
  printf("samples ok=%d/%d, cur[] changed between consecutive samples %d times\n", ok, n, changed);
  for (int j = 0; j < 6; ++j)
    printf("  JT%d cur[A] min %7.3f mean %7.3f max %7.3f | cur_sat max %.3f\n", j + 1, cmin[j],
           ok ? csum[j] / ok : 0.f, cmax[j], smax[j]);
  if (do_zpow) { printf("\n*** ZPOW OFF ***\n"); mon(c, "ZPOW OFF"); sleep(1); }
  memset(&p, 0, sizeof p); krnx_GetPanelInfo(c, 0, &p);
  printf("panel at end: motor=%d error=%d\n", p.motor_lamp, p.error_lamp);
  rc = krnx_Close(c);
  printf("close rc=%d\n", rc);
  return 0;
}
