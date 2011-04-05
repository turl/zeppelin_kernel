/*
 * drivers/power/process.c - Functions for starting/stopping processes on 
 *                           suspend transitions.
 *
 * Originally from swsusp.
 */

#ifdef CONFIG_PM_VERBOSE
#define DEBUG
#endif

#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/freezer.h>
#include <linux/wakelock.h>

/* 
 * Timeout for stopping processes
 */
#define TIMEOUT	(20 * HZ)

static inline int freezeable(struct task_struct * p)
{
	if ((p == current) ||
	    (p->flags & PF_NOFREEZE) ||
	    (p->exit_state != 0))
		return 0;
	return 1;
}

static int try_to_freeze_tasks(bool sig_only)
{
	struct task_struct *g, *p;
	unsigned long end_time;
	unsigned int todo;
	struct timeval start, end;
	u64 elapsed_csecs64;
	unsigned int elapsed_csecs;
	unsigned int wakeup = 0;

	do_gettimeofday(&start);

	end_time = jiffies + TIMEOUT;
	do {
		todo = 0;
		read_lock(&tasklist_lock);
		do_each_thread(g, p) {
			if (frozen(p) || !freezeable(p))
				continue;

			if (!freeze_task(p, sig_only))
				continue;

			/*
			 * Now that we've done set_freeze_flag, don't
			 * perturb a task in TASK_STOPPED or TASK_TRACED.
			 * It is "frozen enough".  If the task does wake
			 * up, it will immediately call try_to_freeze.
			 */
			if (!task_is_stopped_or_traced(p) &&
			    !freezer_should_skip(p))
				todo++;
		} while_each_thread(g, p);
		read_unlock(&tasklist_lock);
		yield();			/* Yield is okay here */
		if (todo && has_wake_lock(WAKE_LOCK_SUSPEND)) {
			wakeup = 1;
			break;
		}
		if (time_after(jiffies, end_time))
			break;
	} while (todo);

	do_gettimeofday(&end);
	elapsed_csecs64 = timeval_to_ns(&end) - timeval_to_ns(&start);
	do_div(elapsed_csecs64, NSEC_PER_SEC / 100);
	elapsed_csecs = elapsed_csecs64;

	if (todo) {
		/* This does not unfreeze processes that are already frozen
		 * (we have slightly ugly calling convention in that respect,
		 * and caller must call thaw_processes() if something fails),
		 * but it cleans up leftover PF_FREEZE requests.
		 */
		if(wakeup) {
			printk("\n");
			printk(KERN_ERR "Freezing of %s aborted\n",
					sig_only ? "user space " : "tasks ");
		}
		else {
			printk("\n");
			printk(KERN_ERR "Freezing of tasks failed after %d.%02d seconds "
					"(%d tasks refusing to freeze):\n",
					elapsed_csecs / 100, elapsed_csecs % 100, todo);
			show_state();
		}
		read_lock(&tasklist_lock);
		do_each_thread(g, p) {
			task_lock(p);
			if (freezing(p) && !freezer_should_skip(p))
				printk(KERN_ERR " %s\n", p->comm);
			cancel_freezing(p);
			task_unlock(p);
		} while_each_thread(g, p);
		read_unlock(&tasklist_lock);
	} else {
		pr_debug("(elapsed %d.%02d seconds) ", elapsed_csecs / 100,
			elapsed_csecs % 100);
	}

	return todo ? -EBUSY : 0;
}

/**
 *	freeze_processes - tell processes to enter the refrigerator
 */
int freeze_processes(void)
{
	int error;

	pr_debug("Freezing user space processes ... ");
	error = try_to_freeze_tasks(true);
	if (error)
		goto Exit;
	pr_debug("done.\n");

	pr_debug("Freezing remaining freezable tasks ... ");
	error = try_to_freeze_tasks(false);
	if (error)
		goto Exit;
	pr_debug("done.");
 Exit:
	BUG_ON(in_atomic());
	pr_debug("\n");
	return error;
}

static void thaw_tasks(bool nosig_only)
{
	struct task_struct *g, *p;

	read_lock(&tasklist_lock);
	do_each_thread(g, p) {
		if (!freezeable(p))
			continue;

		if (nosig_only && should_send_signal(p))
			continue;

		if (cgroup_frozen(p))
			continue;

		thaw_process(p);
	} while_each_thread(g, p);
	read_unlock(&tasklist_lock);
}

void thaw_processes(void)
{
	pr_debug("Restarting tasks ... ");
	thaw_tasks(true);
	thaw_tasks(false);
	schedule();
	pr_debug("done.\n");
}

