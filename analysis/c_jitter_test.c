// Measure per-step jitter of 9x9 complex matvec on CPU.
// Calls lb_propagate_step_soa() N times, recording wall time for each.
// Outputs CSV: trial,ns

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <complex.h>

// Inline 9x9 complex matvec (SoA layout, matches lindblad-bench)
static void matvec_9x9_soa(const double *p_re, const double *p_im,
                            const double *rho_re, const double *rho_im,
                            double *out_re, double *out_im) {
    for (int i = 0; i < 9; i++) {
        double acc_re = 0.0, acc_im = 0.0;
        for (int j = 0; j < 9; j++) {
            int k = i * 9 + j;
            acc_re += p_re[k] * rho_re[j] - p_im[k] * rho_im[j];
            acc_im += p_re[k] * rho_im[j] + p_im[k] * rho_re[j];
        }
        out_re[i] = acc_re;
        out_im[i] = acc_im;
    }
}

static double elapsed_ns(struct timespec *a, struct timespec *b) {
    return (b->tv_sec - a->tv_sec) * 1e9 + (b->tv_nsec - a->tv_nsec);
}

int main(int argc, char **argv) {
    int n_trials = 10000;
    if (argc > 1) n_trials = atoi(argv[1]);

    // Build a simple propagator
    double p_re[81], p_im[81];
    for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++) {
            p_re[i*9+j] = (i == j) ? 0.99 : 0.001 * (i+1) * (j+1);
            p_im[i*9+j] = (i == j) ? 0.01 : 0.0;
        }

    double rho_re[9] = {1.0, 0, 0, 0, 0, 0, 0, 0, 0};
    double rho_im[9] = {0};
    double out_re[9], out_im[9];

    double *times_ns = malloc(n_trials * sizeof(double));

    // Warmup
    for (int i = 0; i < 1000; i++)
        matvec_9x9_soa(p_re, p_im, rho_re, rho_im, out_re, out_im);

    // Timed trials
    struct timespec t0, t1;
    for (int i = 0; i < n_trials; i++) {
        clock_gettime(CLOCK_MONOTONIC, &t0);
        matvec_9x9_soa(p_re, p_im, rho_re, rho_im, out_re, out_im);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        times_ns[i] = elapsed_ns(&t0, &t1);

        // Feed output back as input
        memcpy(rho_re, out_re, sizeof(rho_re));
        memcpy(rho_im, out_im, sizeof(rho_im));
    }

    // Stats
    double sum = 0, sum2 = 0, mn = 1e18, mx = 0;
    for (int i = 0; i < n_trials; i++) {
        sum += times_ns[i];
        sum2 += times_ns[i] * times_ns[i];
        if (times_ns[i] < mn) mn = times_ns[i];
        if (times_ns[i] > mx) mx = times_ns[i];
    }
    double mean = sum / n_trials;
    double stddev = sqrt(sum2 / n_trials - mean * mean);

    fprintf(stderr, "C matvec 9x9 jitter test (%d trials):\n", n_trials);
    fprintf(stderr, "  mean:   %.1f ns\n", mean);
    fprintf(stderr, "  stddev: %.1f ns\n", stddev);
    fprintf(stderr, "  min:    %.1f ns\n", mn);
    fprintf(stderr, "  max:    %.1f ns\n", mx);
    fprintf(stderr, "  range:  %.1f ns (%.1fx mean)\n", mx - mn, (mx - mn) / mean);

    // CSV to stdout
    printf("trial,ns\n");
    for (int i = 0; i < n_trials; i++)
        printf("%d,%.1f\n", i, times_ns[i]);

    free(times_ns);
    return 0;
}
