/* KRNX controller probe (not built by CMake; hand-compile when needed).
 *
 *   cd khi_hardware && g++ -O0 -o /tmp/probe tools/krnx_probe.cpp -Iinclude -Llib/x86_64 -lkrnx \
 *       -Wl,-rpath,$PWD/lib/x86_64
 *
 *
 *   probe <ip>          READ-ONLY: status structs + read-only monitor commands.
 *   probe <ip> read <n> READ-ONLY as KRNX controller number <n> (port set from
 *                       port_ref.tbl / port_def.tbl in the working directory).
 *   probe <ip> zpow     ENERGIZES MOTORS: sends "ZPOW ON" through the monitor
 *                       channel (the same AS command the driver's activate uses),
 *                       prints the controller's verbatim reply, polls the motor
 *                       lamp for up to 10 s, then sends "ZPOW OFF".
 *
 * Only run "zpow" with the arm idle, nobody in the cell, and the driver down.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "khi_hardware/krnx.h"

#define PROBE_BUF 16384

static int mon(int c, const char * cmd)
{
  char buf[PROBE_BUF];
  int as = 0;
  memset(buf, 0, sizeof buf);
  int rc = krnx_ExecMon(c, cmd, buf, sizeof buf, &as);
  printf("---- ExecMon \"%s\" rc=%d (-0x%X) as_err=%d ----\n%s\n", cmd, rc, -rc, as, buf);
  return rc;
}

static void panel(int c, const char * tag)
{
  TKrnxPanelInfo p;
  memset(&p, 0, sizeof p);
  int rc = krnx_GetPanelInfo(c, 0, &p);
  printf(
    "%s PanelInfo rc=%d: error=%d motor=%d cycle=%d repeat=%d run=%d trigger=%d teach_lock=%d "
    "emergency=%d\n",
    tag, rc, p.error_lamp, p.motor_lamp, p.cycle_lamp, p.repeat_lamp, p.run_lamp, p.trigger_lamp,
    p.teach_lock_lamp, p.emergency);
}

int main(int argc, char ** argv)
{
  const char * ip = argc > 1 ? argv[1] : "192.168.1.24";
  const int do_zpow = argc > 2 && strcmp(argv[2], "zpow") == 0;
  /* Optional third argument: KRNX controller number (default 0). KRNX maps the
   * controller number to a PORT SET through port_ref.tbl / port_def.tbl in the
   * working directory (defaults when absent), so a second physical E-controller
   * opened as controller 1 needs tables that give record 1 the E-controller
   * ports too; this lets that be checked before the driver is configured. */
  const int cont_no = argc > 3 ? atoi(argv[3]) : 0;
  char host[64];
  strncpy(host, ip, 63);
  host[63] = 0;
  TKrnxOpenErrorInfo ei;
  memset(&ei, 0, sizeof ei);
  int c = krnx_OpenSingleProcess(cont_no, host, &ei);
  printf("open cont_no=%d rc=%d (err_no=%d port=%d pid=%d src=%s)\n", cont_no, c, ei.err_no, ei.port, ei.pid, ei.source);
  if (c < 0) return 1;
  krnx_SetAuxApiTimeoutPeriod(c, 3000);

  panel(c, "initial");
  int ec = -1;
  int rc = krnx_GetCurErrorInfo(c, 0, &ec);
  printf("CurErrorInfo rc=%d code=%d\n", rc, ec);

  if (!do_zpow)
  {
    /* Read-only, non-interactive, non-refreshing monitor commands only. */
    const char * cmds[] = {"STATUS", "SWITCH", "ERRLOG", "RTC_STATUS", "WHERE", "ID",
                           "SIGNAL",  "FREE",   NULL};
    for (int i = 0; cmds[i]; ++i) mon(c, cmds[i]);
  }
  else
  {
    printf("\n*** ENERGIZING MOTORS: sending ZPOW ON ***\n");
    mon(c, "ZPOW ON");
    int lit = 0;
    for (int i = 0; i < 100; ++i)
    {
      TKrnxPanelInfo p;
      memset(&p, 0, sizeof p);
      krnx_GetPanelInfo(c, 0, &p);
      if (p.motor_lamp)
      {
        printf("motor lamp ON after ~%d ms\n", i * 100);
        lit = 1;
        break;
      }
      usleep(100000);
    }
    if (!lit) printf("motor lamp still OFF after 10 s\n");
    panel(c, "after ZPOW ON");
    ec = -1;
    krnx_GetCurErrorInfo(c, 0, &ec);
    printf("CurErrorInfo after ZPOW ON: code=%d\n", ec);
    mon(c, "SWITCH");
    mon(c, "ERRLOG");
    printf("\n*** sending ZPOW OFF ***\n");
    mon(c, "ZPOW OFF");
    sleep(1);
    panel(c, "after ZPOW OFF");
  }

  rc = krnx_Close(c);
  printf("close rc=%d\n", rc);
  return 0;
}
