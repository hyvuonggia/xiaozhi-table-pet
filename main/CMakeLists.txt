idf_component_register(SRCS "main.c" "audio_in.c" "audio_out.c"
                            "msg_filter.c" "servo.c" "buttons.c" "voice_path.c"
                            "display.c" "watchdog.c" "wakeword.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES nvs_flash esp_wifi esp_netif esp_event
                                  esp_http_client esp_websocket_client esp_http_server
                                  json esp_timer driver
                                  espressif__esp_audio_codec espressif__esp-dsp
                                  espressif__esp-sr)
