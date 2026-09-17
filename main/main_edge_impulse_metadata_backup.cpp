#include <cstdio>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"

extern "C" void app_main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("P-101 / EDGE GUARD\n");
    printf("Teste de integracao Edge Impulse\n");
    printf("========================================\n");

    printf(
        "Frequencia........: %.0f Hz\n",
        (double)EI_CLASSIFIER_FREQUENCY
    );

    printf(
        "Intervalo.........: %.2f ms\n",
        (double)EI_CLASSIFIER_INTERVAL_MS
    );

    printf(
        "Amostras/janela...: %d\n",
        EI_CLASSIFIER_RAW_SAMPLE_COUNT
    );

    printf(
        "Eixos por frame...: %d\n",
        EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME
    );

    printf(
        "DSP input.........: %d valores\n",
        EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
    );

    printf(
        "Classes...........: %d\n",
        EI_CLASSIFIER_LABEL_COUNT
    );

    printf("\nLabels do modelo:\n");

    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        printf(
            "  [%u] %s\n",
            (unsigned)i,
            ei_classifier_inferencing_categories[i]
        );
    }

    printf("\n");
    printf("Edge Impulse SDK carregado com sucesso.\n");
    printf("========================================\n");
}