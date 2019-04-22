/*
 * iperf2 TCP-Client for WLAN-Speed measurement
 * for HelTec WIFI_Kit_32 / LoRA ESP32 Board with OLED
 */

#include "heltec.h"
#include "WiFi.h"
#include "my_logo.h"
#include <sys/socket.h>

const char* ssid = "xxxxxx"; //replace "xxxxxx" with your WIFI's ssid
const char* password = "xxxxxxxx"; //replace "xxxxxx" with your WIFI's password
const char* server_ip = "192.168.x.x";

#define IPERF_TRAFFIC_TASK_NAME "iperf_traffic"
#define IPERF_TRAFFIC_TASK_PRIORITY 10
#define IPERF_TRAFFIC_TASK_STACK 4096

#define IPERF_REPORT_TASK_NAME "iperf_report"
#define IPERF_REPORT_TASK_STACK 4096
#define IPERF_REPORT_TASK_PRIORITY 20
#define IPERF_TCP_TX_LEN (16 << 10)

#define IPERF_DEFAULT_PORT 5001
#define IPERF_DEFAULT_INTERVAL 1
#define IPERF_DEFAULT_TIME 5

#define NORMAL_LOW  (0) // Input PIN beavior
#define NORMAL_HIGH (1)

typedef struct {
    uint32_t dip;
    uint32_t sip;
    uint16_t dport;
    uint16_t sport;
    uint32_t interval;
    uint32_t time;
} iperf_cfg_t;

typedef struct {
    iperf_cfg_t cfg;
    bool        finish;
    uint32_t    total_len;
    uint32_t    buffer_len;
    uint8_t     *buffer;
    uint32_t    sockfd;
} iperf_ctrl_t;


static bool         s_iperf_is_running = false;
static iperf_ctrl_t s_iperf_ctrl;
static double       bw=0.0;
static int          m_btnPin=0;

static const char *TAG = "iperf";

/**************************************************************************************************/
void(* resetFunc) (void) = 0;  // Emulates a Cold Reset

/**************************************************************************************************/
static int iperf_get_socket_error_code(int sockfd)
{
    uint32_t optlen = sizeof(int);
    int result;
    int err;

    /* get the error state, and clear it */
    err = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &result, &optlen);
    if (err == -1) {
        ESP_LOGE(TAG, "getsockopt failed: ret=%d", err);
        return -1;
    }

    return result;
}

/**************************************************************************************************/
static int iperf_show_socket_error_reason(const char *str, int sockfd)
{
    int err = iperf_get_socket_error_code(sockfd);

    if (err != 0) {
        ESP_LOGW(TAG, "%s error, error code: %d, reason: %s", str, err, strerror(err));
    }
    resetFunc();  //call reset
    return err;
}

/**************************************************************************************************/
static void iperf_report_task(void *arg)
{
    uint32_t interval = s_iperf_ctrl.cfg.interval;
    uint32_t time = s_iperf_ctrl.cfg.time;
    TickType_t delay_interval = (interval * 1000) / portTICK_PERIOD_MS;
    uint32_t last_len = 0;
    uint32_t cur = 0;
    
    printf("\n%16s %s\n", "Interval", "Bandwidth");
    while (!s_iperf_ctrl.finish) {
        vTaskDelay(delay_interval);

        bw=(double)((s_iperf_ctrl.total_len - last_len) * 8) / interval / 1e6;
        Heltec.display -> clear();
        Heltec.display -> drawString(0, 40, "Bw: "+(String)(bw)+" Mbit/s");
        Heltec.display -> display();
        printf("%4d-%4d sec       %.2f Mbits/sec\n", cur, cur + interval,bw);
        
        cur += interval;
        last_len = s_iperf_ctrl.total_len;
        if (cur == time) break;  // End Time reached?, Exit while
    }

    if (cur != 0) { // got some data?
      
        bw=(double)(s_iperf_ctrl.total_len * 8) / cur / 1e6;
        Heltec.display -> clear();
        Heltec.display -> drawString(0, 40, "Bw: "+(String)(bw)+" Mbit/s");
        Heltec.display -> display();
        printf("%4d-%4d sec       %.2f Mbits/sec\n", 5, time, bw);

    }

    s_iperf_ctrl.finish = true;
    vTaskDelete(NULL);
}

/**************************************************************************************************/
static esp_err_t iperf_start_report(void)
{
    int ret;

    ret = xTaskCreatePinnedToCore(iperf_report_task, IPERF_REPORT_TASK_NAME, IPERF_REPORT_TASK_STACK, NULL, IPERF_REPORT_TASK_PRIORITY, NULL, portNUM_PROCESSORS - 1);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "create task %s failed", IPERF_REPORT_TASK_NAME);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**************************************************************************************************/
static esp_err_t iperf_run_tcp_client(void)
{
    struct sockaddr_in remote_addr;
    int actual_send = 0;
    int want_send = 0;
    uint8_t *buffer;
    int sockfd;
    
    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd < 0) {
        iperf_show_socket_error_reason("tcp client create", sockfd);
        return ESP_FAIL;
    }

    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(s_iperf_ctrl.cfg.dport);
    remote_addr.sin_addr.s_addr = s_iperf_ctrl.cfg.dip;
    if (connect(sockfd, (struct sockaddr *)&remote_addr, sizeof(remote_addr)) < 0) {
        iperf_show_socket_error_reason("tcp client connect", sockfd);
        return ESP_FAIL;
    }

    iperf_start_report(); // start measuring loop
    
    buffer = s_iperf_ctrl.buffer;
    want_send = s_iperf_ctrl.buffer_len;
    while (!s_iperf_ctrl.finish) {
        actual_send = send(sockfd, buffer, want_send, 0);
        if (actual_send <= 0) {
            iperf_show_socket_error_reason("tcp client send", sockfd);
            break;
        } else {
            s_iperf_ctrl.total_len += actual_send;
        }
        //Serial.println("actual_send:" + String(s_iperf_ctrl.total_len));
    }

    close(sockfd);
    return ESP_OK;
}

/**************************************************************************************************/
static void iperf_task_traffic(void *arg)
{
    iperf_run_tcp_client();
    
    if (s_iperf_ctrl.buffer) {
        free(s_iperf_ctrl.buffer);
        s_iperf_ctrl.buffer = NULL;
    }
    s_iperf_is_running = false;
    vTaskDelete(NULL);
}

/**************************************************************************************************/
esp_err_t iperf_start(iperf_cfg_t *cfg)
{
    BaseType_t ret;

    if (!cfg) { return ESP_FAIL; }

    if (s_iperf_is_running) {
        ESP_LOGW(TAG, "iperf is running");
        return ESP_FAIL;
    }

    memset(&s_iperf_ctrl, 0, sizeof(s_iperf_ctrl));
    memcpy(&s_iperf_ctrl.cfg, cfg, sizeof(*cfg));
    s_iperf_is_running  = true;
    s_iperf_ctrl.finish = false;
    s_iperf_ctrl.buffer_len = IPERF_TCP_TX_LEN;
    s_iperf_ctrl.buffer = (uint8_t *)malloc(s_iperf_ctrl.buffer_len);
    if (!s_iperf_ctrl.buffer) {
        ESP_LOGE(TAG, "create buffer: not enough memory");
        return ESP_FAIL;
    }
    memset(s_iperf_ctrl.buffer, 0, s_iperf_ctrl.buffer_len);

    ret = xTaskCreatePinnedToCore(iperf_task_traffic, IPERF_TRAFFIC_TASK_NAME, IPERF_TRAFFIC_TASK_STACK, NULL, IPERF_TRAFFIC_TASK_PRIORITY, NULL, portNUM_PROCESSORS - 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "create task %s failed", IPERF_TRAFFIC_TASK_NAME);
        free(s_iperf_ctrl.buffer);
        s_iperf_ctrl.buffer = NULL;
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**************************************************************************************************/
static int wifi_cmd_iperf()
{
    uint32_t ipBuffer;
    iperf_cfg_t cfg;

    memset(&cfg, 0, sizeof(cfg));

    cfg.sip      = WiFi.localIP();       // get my address
    inet_aton(server_ip,&ipBuffer); 
    cfg.dip      = ipBuffer;             // server address
    cfg.sport    = IPERF_DEFAULT_PORT;
    cfg.dport    = IPERF_DEFAULT_PORT;
    cfg.interval = IPERF_DEFAULT_INTERVAL;  
    cfg.time     = IPERF_DEFAULT_TIME; 

    ESP_LOGI(TAG, "sip=%d.%d.%d.%d:%d, dip=%d.%d.%d.%d:%d, int=%d, tim=%d",
            cfg.sip&0xFF, (cfg.sip>>8)&0xFF, (cfg.sip>>16)&0xFF, (cfg.sip>>24)&0xFF, cfg.sport,
            cfg.dip&0xFF, (cfg.dip>>8)&0xFF, (cfg.dip>>16)&0xFF, (cfg.dip>>24)&0xFF, cfg.dport,
            cfg.interval, 
            cfg.time);

    iperf_start(&cfg);

    return 0;
}

/**************************************************************************************************/
void logo(){
	Heltec.display -> clear();
	Heltec.display -> drawXbm(0,5,logo_width,logo_height,(const unsigned char *)logo_bits);
	Heltec.display -> display();
}

/**************************************************************************************************/
void WIFISetUp(void)
{
	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	WiFi.disconnect(true);
	delay(1000);
	WiFi.mode(WIFI_STA);
	WiFi.setAutoConnect(true);
	WiFi.begin(ssid, password);
	delay(100);

	byte count = 0;
	while(WiFi.status() != WL_CONNECTED && count < 10)
	{
		count ++;
		delay(500);
		Heltec.display -> drawString(0, 0, "Connecting...");
		Heltec.display -> display();
	}

	Heltec.display -> clear();
	if(WiFi.status() == WL_CONNECTED)
	{
		Heltec.display -> drawString(0, 0, "Connecting...OK.");
		Heltec.display -> display();
		delay(500);
	}
	else
	{
		Heltec.display -> clear();
		Heltec.display -> drawString(0, 0, "Connecting...Failed");
		Heltec.display -> display();
    delay(3000);
    resetFunc();  //call reset
	}
	Heltec.display -> drawString(0, 20, "WIFI Setup done");
	Heltec.display -> display();
	delay(500);
}

/**************************************************************************************************/
void setup()
{
	pinMode(LED,OUTPUT);
  // digitalWrite(LED,HIGH);   // LED ein
  digitalWrite(LED,LOW);    // LED aus

  pinMode(m_btnPin, INPUT_PULLUP);

	Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);

	logo();
	delay(300);
	Heltec.display -> clear();
  Heltec.display -> setFont(ArialMT_Plain_16);
      
	WIFISetUp();
}

/**************************************************************************************************/
void loop()
{
  do {

      if (WiFi.status() == WL_CONNECTED)
        {
          wifi_cmd_iperf(); 
          ESP_LOGI(TAG, "iperf startet");
        }

      while (!s_iperf_ctrl.finish) {
       delay(1000);  // wait 1 second
      }

      ESP_LOGI(TAG, "iperf stopped");
      
      Heltec.display -> drawString(0, 0, "Press PRG/BOOT");
      Heltec.display -> display();

      while (digitalRead(m_btnPin) == NORMAL_HIGH) { // is the button down?
       delay(100);
      }
      Heltec.display -> clear();
      Heltec.display -> display();
      
  } while (1);

}
