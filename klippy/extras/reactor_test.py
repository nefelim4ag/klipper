import math
import logging

class TestReactor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.test_timers = {}
        self.scheduled = {}
        self.lag = {}
        self.counter = {}
        # per-timer Welford state and global Welford
        self.stats = {
            "_global": {"count": 0, "mean": 0.0, "M2": 0.0}
        }
        for i in range(20):
            self.lag[i] = .0
            self.counter[i] = 0
            step = 0.05 + 0.01 * i

            def timer_event(eventtime, indx=i, step=step):
                now = self.reactor.monotonic()
                sample = now - self.scheduled[indx]
                self.lag[indx] += sample
                self.counter[indx] += 1
                # logging.info(f"Timer {indx}: running")

                # Stats magic
                self.stats["_global"]["count"] += 1
                delta = sample - self.stats["_global"]["mean"]
                self.stats["_global"]["mean"] += delta / self.stats["_global"]["count"]
                self.stats["_global"]["M2"] += delta * (sample - self.stats["_global"]["mean"])

                # Do something
                self.scheduled[indx] = eventtime + step
                if (self.counter[indx] + indx) % 4 == 0:
                    self.reactor.pause(self.reactor.monotonic() + 0.001)
                for i in range(100*indx):
                    pass
                if self.counter[indx] > 300:
                    return self.reactor.NEVER
                return self.scheduled[indx]

            self.scheduled[i] = self.reactor.monotonic() + step
            self.test_timers[i] = self.reactor.register_timer(timer_event, self.scheduled[i])

        self.reactor.pause(self.reactor.monotonic() + 15)
        cummulative_lag = .0
        max_lag = .0
        min_lag = 9999.
        for i in range(len(self.test_timers)):
            if self.counter[i] == 0:
                logging.error(f"What? {i}")
                raise
            avg_lag = self.lag[i] / self.counter[i]
            max_lag = max(avg_lag, max_lag)
            min_lag = min(avg_lag, min_lag)
            cummulative_lag += avg_lag
        avg_lag = cummulative_lag / len(self.counter)
        logging.info("Cummulative Lag: %.9f, min: %.9f, max: %.9f", avg_lag, min_lag, max_lag)

        g_mean = self.stats["_global"]["mean"]
        g = self.stats["_global"]
        g_std = math.sqrt(g["M2"] / (g["count"] - 1))
        se = g_std / math.sqrt(g["count"])
        ci95 = 1.96 * se
        logging.info("mean=%.9f std=%.9f 95%% Confidence Interval: ±%.9f (n=%d)" %
            (g_mean, g_std, ci95, g["count"]))

def load_config(config):
    return TestReactor(config)
