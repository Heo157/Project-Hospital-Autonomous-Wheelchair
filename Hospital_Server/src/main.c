/**
 * ============================================================================
 * @file    main.c
 * @brief   C ��� ��� �߰� ���� (Protocol Aware)
 * @details common_defs.h�� ���ǵ� ���������� �ؼ��Ͽ� �α׸� ����ϴ� ����
 * Qt Ŭ���̾�Ʈ�� ROS �κ� ������ ����� �߰��ϱ� ���� ���� �����Դϴ�.
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>      // close(), fork() ���� ����ϱ� ���� �ʿ�
#include <arpa/inet.h>   // sockaddr_in, inet_ntoa() �� ��Ʈ��ũ �ּ� ����
#include <sys/socket.h>  // socket(), bind(), accept() �� ���� �ٽ� �Լ�
#include <sys/types.h>
#include <sys/wait.h>    // waitpid()�� ����ϱ� ���� �ʿ� (���� ���μ��� ó��)
#include <signal.h>      // signal() �Լ� ���
#include "server_db.h"

 // �츮�� ������ ���� ��� (��Ŷ ����ü, ��� ��)
 // �� ������ Ŭ���̾�Ʈ(Qt/ROS)�� ������ �ݵ�� ������ ������ ������ �־�� �մϴ�.
#include "../include/common_defs.h"

/**
 * @brief ���� ���μ���(Zombie Process) ó�� �ڵ鷯
 * * [���� ����]
 * fork()�� ���� ������ �ڽ� ���μ����� �� ���� �� �ϰ� ����(exit)�ϸ�,
 * �θ� ���μ����� �� ���� ���¸� Ȯ������ ������ Ŀ�ο� '����' ���·� ���� ���ҽ��� �����մϴ�.
 * �̸� �����ϱ� ���� �ڽ��� �׾��ٴ� ��ȣ(SIGCHLD)�� ���� waitpid�� û�����ݴϴ�.
 */
void handle_sigchld(int sig) {
    (void)sig; // �����Ϸ��� '������ ���� ����' ��� ����

    // waitpid(-1, ...): ����� ������ �ڽ� ���μ����� ��ٸ�
    // WNOHANG: �ڽ� ���μ����� ���� ������� �ʾ����� ��ٸ��� �ʰ� �ٷ� ���� (Non-blocking)
    while (waitpid(-1, NULL, WNOHANG) > 0);
}

/**
 * @brief Ŭ���̾�Ʈ 1:1 ���� ��ũ �Լ� (�ٽ� ����)
 * * [���� ����]
 * �� �Լ��� fork()�� �ڽ� ���μ������� ����˴ϴ�.
 * ����� �ϳ��� Ŭ���̾�Ʈ�� ��� ��ȭ�ϸ� ��Ŷ�� �ְ��޽��ϴ�.
 */
void handle_client(int client_sock, struct sockaddr_in client_addr) {
    PacketHeader header;           // ��Ŷ�� �Ӹ�(Header)�� ���� ����ü
    char buffer[MAX_BUFFER_SIZE];  // ��Ŷ�� ����(Payload)�� ���� ����
    ssize_t read_len;              // �о���� ����Ʈ ��

    // ������ Ŭ���̾�Ʈ�� IP �ּҸ� ���ڿ��� ��ȯ�Ͽ� ���
    printf("[Server] New client connected: %s\n", inet_ntoa(client_addr.sin_addr));

    while (1) {
        // ====================================================================
        // 1�ܰ�: ��� �б� (���� ���� 4����Ʈ)
        // ====================================================================
        // MSG_WAITALL: TCP�� �����Ͱ� ��Ʈ��(���ٱ�)ó�� �帣�� ������,
        // 4����Ʈ�� ��û�ص� ��Ʈ��ũ ��Ȳ�� ���� 1~2����Ʈ�� �� �� �ֽ��ϴ�.
        // �� �ɼ��� 4����Ʈ�� �� �� ������ ��ٷȴٰ� �����϶�� �����մϴ�.
        read_len = recv(client_sock, &header, sizeof(PacketHeader), MSG_WAITALL);

        if (read_len <= 0) {
            // 0�̸� Ŭ���̾�Ʈ�� ���� ����(close), -1�̸� ���� �߻�
            printf("[Server] Client disconnected or error.\n");
            break; // while ���� Ż�� -> �Լ� ���� -> ���μ��� ����
        }

        // ====================================================================
        // 2�ܰ�: ��ȿ�� �˻� (Magic Number)
        // ====================================================================
        // �����Ͱ� �߰��� �����ų�, �츮 ���������� �𸣴� �̻��� ���� �����ߴ��� Ȯ���մϴ�.
        // common_defs.h�� ���ǵ� MAGIC_NUMBER(0xAB)�� �ٸ��� �����մϴ�.
        if (header.magic != MAGIC_NUMBER) {
            printf("[Warning] Invalid magic number: 0x%02X\n", header.magic);
            continue; // �̹� ��Ŷ�� ������ ���� ��Ŷ ���
        }

        // ====================================================================
        // 3�ܰ�: ���̷ε�(Payload) �б�
        // ====================================================================
        // ����� ���� payload_len(������ ����)��ŭ �߰��� �����͸� �о�ɴϴ�.
        if (header.payload_len > 0) {
            read_len = recv(client_sock, buffer, header.payload_len, MSG_WAITALL);

            // ������� 10����Ʈ �����ٰ� �ߴµ�, �����δ� �� �Դٸ�? -> ���� ó��
            if (read_len != header.payload_len) {
                printf("[Error] Payload read mismatch.\n");
                break;
            }
        }

        // ====================================================================
        // 4�ܰ�: �޽��� �ؼ� �� ó��
        // ====================================================================
        // ������� ������ ������ ��Ŷ �ϳ�(Header + Payload)�� �տ� �� ���Դϴ�.
        printf("[Packet] Device: 0x%02X | Type: 0x%02X | Len: %d -> ",
            header.device_type, header.msg_type, header.payload_len);

        switch (header.msg_type) {
        case MSG_LOGIN_REQ:
            printf("Login Request received.\n");
            // ���߿��� ���⼭ DB�� ��ȸ�ϰų� �α��� ���� ��Ŷ�� ������ ��
            break;

        case MSG_MOVE_FORWARD:
            printf("Command: MOVE FORWARD\n");
            // ���߿��� �� ������ ����� ROS �κ� ���μ������� �����ؾ� �� (IPC �ʿ�)
            break;

        case MSG_MOVE_BACKWARD:
            printf("Command: MOVE BACKWARD\n");
            break;

        case MSG_STOP:
            printf("Command: STOP!\n");
            break;

        case MSG_MOVE_TO_GOAL:
            // ���̷ε带 WaypointData ����ü ������� �ؼ�(Casting)
            if (header.payload_len == sizeof(WaypointData)) {
                WaypointData* wp = (WaypointData*)buffer;
                printf("Goal: X=%.2f, Y=%.2f, Theta=%.2f\n", wp->x, wp->y, wp->theta);
            }
            break;

        case MSG_ROBOT_STATE:
            // ���̷ε带 RobotStateData ����ü ������� �ؼ�
            if (header.payload_len == sizeof(RobotStateData)) {
                RobotStateData* st = (RobotStateData*)buffer;
                printf("State: Battery=%d%%, Pos=(%.2f, %.2f)\n",
                    st->battery_level, st->current_x, st->current_y);

                if (robot_name[0] == '\0') {
                    printf("[DB] robot_name empty. Send MSG_LOGIN_REQ first.\n");
                    break;
                }

                const char *op = (st->is_moving ? "RUNNING" : "STOP");

                int batt_i = st->battery_level;
                if (batt_i < 0) batt_i = 0;
                if (batt_i > 100) batt_i = 100;

                uint32_t batt = (uint32_t)batt_i;
                uint8_t charging = 0; // 프로토콜에 충전 정보 없으면 0 고정

                if (db_ok) {
                    db_upsert_robot_status(&db, robot_name, op, batt, charging,
                                           (double)st->current_x, (double)st->current_y);
                }
            
            }

            
            break;

        default:
            printf("Unknown Message Type.\n");
            break;
        }
    }

    // while ������ ���������� ������ �ݰ� ���μ����� �����մϴ�.
    close(client_sock);
    exit(0); // �ڽ� ���μ��� ���� (�̶� SIGCHLD ��ȣ�� �θ𿡰� ���ư�)
}

int main() {
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    // [Signal ���] �ڽ� ���μ����� ������ handle_sigchld �Լ� ����
    signal(SIGCHLD, handle_sigchld);

    // 1. ���� ���� (IPv4, TCP ��Ʈ�� ���)
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock == -1) {
        perror("socket error");
        return 1;
    }

    // [�߿� �ɼ�] "Address already in use" ���� ����
    // ������ ���� �����ϰ� �ٷ� �ٽ� �� ��, ��Ʈ�� ���� ���� ���̶�� ������ ���� ���� �����ݴϴ�.
    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 2. �ּ� ���� (IP�� Port ����)
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // �� ��ǻ���� ��� IP�� ���� ���
    server_addr.sin_port = htons(SERVER_PORT);       // common_defs.h�� ���ǵ� ��Ʈ (8080)

    // 3. Bind (���Ͽ� �ּ�ǥ ���̱�)
    if (bind(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        perror("bind error");
        return 1;
    }

    // 4. Listen (���� ��⿭ ����, �ִ� 5�� ��� ����)
    if (listen(server_sock, 5) == -1) {
        perror("listen error");
        return 1;
    }

    printf("=== Middle Server Started on Port %d ===\n", SERVER_PORT);

    while (1) {
        // 5. Accept (Ŭ���̾�Ʈ�� �� ������ ����ŷ/���)
        // ������ �����ϸ� ���ο� ����(client_sock)�� ������ݴϴ�.
        client_sock = accept(server_sock, (struct sockaddr*)&client_addr, &addr_len);
        if (client_sock == -1) {
            continue; // ���� ���� �ٽ� ���
        }

        // 6. Fork (���μ��� ���� - ���� ó��)
        // �θ�(���� ��ü)�� ��� ���ο� �մ��� �޾ƾ� �ϹǷ�,
        // ���� ���� �մ��� �����ΰ�(�ڽ� ���μ���)���� �ñ�ϴ�.
        pid_t pid = fork();

        if (pid == 0) {
            // [�ڽ� ���μ��� ����]
            // �ڽ��� ���� ����(�Ա�)�� �ʿ� �����ϴ�.
            close(server_sock);
            // Ŭ���̾�Ʈ ��� �Լ� ���� (���⼭ ���ѷ��� ���� ��ȭ��)
            handle_client(client_sock, client_addr);
        }
        else if (pid > 0) {
            // [�θ� ���μ��� ����]
            // �θ�� Ŭ���̾�Ʈ�� ��ȭ�� �ʿ䰡 �����ϴ�(�ڽ��� �ϴϱ�).
            // ���� Ŭ���̾�Ʈ ���� �ڵ鸸 �ݰ�, �ٽ� accept()�Ϸ� ���� ���� �ö󰩴ϴ�.
            close(client_sock);
        }
        else {
            perror("fork error");
        }
    }

    // ���� ���� �� ���� ���� �ݱ� (��ǻ� �������� ����)
    if (db_ok) db_close(&db);
    close(server_sock);
    return 0;
}