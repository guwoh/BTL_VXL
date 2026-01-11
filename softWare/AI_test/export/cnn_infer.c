#include <stdio.h>
#include <math.h>

#include "conv1.h"
#include "conv2.h"
#include "dense1.h"
#include "dense2.h"

#define INPUT_LEN 50
#define INPUT_C   6

/* ------------------ Conv1D ------------------ */
void conv1d(
    const float *input,
    int L, int Cin, int Cout, int K,
    const int8_t *w,
    const int32_t *b,
    float w_scale,
    float *output
) {
    int outL = L - K + 1;

    for (int t = 0; t < outL; t++) {
        for (int co = 0; co < Cout; co++) {
            float sum = (float)b[co];

            for (int k = 0; k < K; k++) {
                for (int ci = 0; ci < Cin; ci++) {
                    int w_idx = (k*Cin + ci)*Cout + co;
                    float wf = w[w_idx] * w_scale;
                    sum += input[(t+k)*Cin + ci] * wf;
                }
            }

            /* ReLU */
            if (sum < 0) sum = 0;
            output[t*Cout + co] = sum;
        }
    }
}

/* ------------------ Global Average Pooling ------------------ */
void gap(const float *in, int L, int C, float *out) {
    for (int c = 0; c < C; c++) {
        float s = 0;
        for (int t = 0; t < L; t++)
            s += in[t*C + c];
        out[c] = s / L;
    }
}

/* ------------------ Dense ------------------ */
void dense(
    const float *in,
    int Cin, int Cout,
    const int8_t *w,
    const int32_t *b,
    float w_scale,
    float *out
) {
    for (int o = 0; o < Cout; o++) {
        float s = b[o];
        for (int i = 0; i < Cin; i++)
            s += in[i] * (w[i*Cout + o] * w_scale);

        if (s < 0) s = 0;   // ReLU
        out[o] = s;
    }
}

/* ------------------ MAIN ------------------ */
int main() {
    /* dummy input */
    float input[INPUT_LEN][INPUT_C] = {0};

    static float conv1_out[INPUT_LEN][CONV1_COUT];
    static float conv2_out[INPUT_LEN][CONV2_COUT];
    static float gap_out[CONV2_COUT];
    static float dense1_out[DENSE1_COUT];
    static float dense2_out[DENSE2_COUT];

    /* Conv1 */
    conv1d(
        &input[0][0], INPUT_LEN, INPUT_C,
        CONV1_COUT, CONV1_K,
        &conv1_w[0][0][0], conv1_b, CONV1_WSCALE,
        &conv1_out[0][0]
    );

    int L1 = INPUT_LEN - CONV1_K + 1;

    /* Conv2 */
    conv1d(
        &conv1_out[0][0], L1, CONV1_COUT,
        CONV2_COUT, CONV2_K,
        &conv2_w[0][0][0], conv2_b, CONV2_WSCALE,
        &conv2_out[0][0]
    );

    int L2 = L1 - CONV2_K + 1;

    /* GAP */
    gap(&conv2_out[0][0], L2, CONV2_COUT, gap_out);

    /* Dense1 */
    dense(gap_out, DENSE1_CIN, DENSE1_COUT,
          &dense1_w[0][0], dense1_b, DENSE1_WSCALE,
          dense1_out);

    /* Dense2 (no ReLU for logits) */
    for (int o = 0; o < DENSE2_COUT; o++) {
        float s = dense2_b[o];
        for (int i = 0; i < DENSE2_CIN; i++)
            s += dense1_out[i] * (dense2_w[i][o] * DENSE2_WSCALE);
        dense2_out[o] = s;
    }

    /* Argmax */
    int best = 0;
    for (int i = 1; i < DENSE2_COUT; i++)
        if (dense2_out[i] > dense2_out[best]) best = i;

    printf("Predicted class = %d\n", best);
    return 0;
}
