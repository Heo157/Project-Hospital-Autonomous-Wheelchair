/*M!999999\- enable the sandbox mode */ 
-- MariaDB dump 10.19-11.8.3-MariaDB, for debian-linux-gnu (aarch64)
--
-- Host: localhost    Database: hospital_db
-- ------------------------------------------------------
-- Server version	11.8.3-MariaDB-0+deb13u1 from Debian

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*M!100616 SET @OLD_NOTE_VERBOSITY=@@NOTE_VERBOSITY, NOTE_VERBOSITY=0 */;

--
-- Table structure for table `TB_BED`
--

DROP TABLE IF EXISTS `TB_BED`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `TB_BED` (
  `ward_name` varchar(50) NOT NULL,
  `bed_num` int(11) NOT NULL,
  PRIMARY KEY (`ward_name`,`bed_num`),
  CONSTRAINT `TB_BED_ibfk_1` FOREIGN KEY (`ward_name`) REFERENCES `TB_WARD` (`ward_name`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `TB_BED`
--

LOCK TABLES `TB_BED` WRITE;
/*!40000 ALTER TABLE `TB_BED` DISABLE KEYS */;
set autocommit=0;
INSERT INTO `TB_BED` VALUES
('5동',1),
('5동',2),
('5동',3),
('5동',4),
('5동',5),
('5동',6);
/*!40000 ALTER TABLE `TB_BED` ENABLE KEYS */;
UNLOCK TABLES;
commit;

--
-- Table structure for table `TB_WARD`
--

DROP TABLE IF EXISTS `TB_WARD`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `TB_WARD` (
  `ward_name` varchar(50) NOT NULL,
  PRIMARY KEY (`ward_name`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `TB_WARD`
--

LOCK TABLES `TB_WARD` WRITE;
/*!40000 ALTER TABLE `TB_WARD` DISABLE KEYS */;
set autocommit=0;
INSERT INTO `TB_WARD` VALUES
('5동');
/*!40000 ALTER TABLE `TB_WARD` ENABLE KEYS */;
UNLOCK TABLES;
commit;

--
-- Table structure for table `call_queue`
--

DROP TABLE IF EXISTS `call_queue`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `call_queue` (
  `call_id` int(11) NOT NULL AUTO_INCREMENT,
  `call_time` datetime DEFAULT current_timestamp(),
  `caller_name` varchar(50) DEFAULT NULL,
  `start_loc` varchar(50) DEFAULT NULL,
  `dest_loc` varchar(50) DEFAULT NULL,
  `is_dispatched` int(11) DEFAULT 0,
  `eta` varchar(50) DEFAULT NULL,
  PRIMARY KEY (`call_id`)
) ENGINE=InnoDB AUTO_INCREMENT=90 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `call_queue`
--

LOCK TABLES `call_queue` WRITE;
/*!40000 ALTER TABLE `call_queue` DISABLE KEYS */;
set autocommit=0;
/*!40000 ALTER TABLE `call_queue` ENABLE KEYS */;
UNLOCK TABLES;
commit;

--
-- Table structure for table `disease_types`
--

DROP TABLE IF EXISTS `disease_types`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `disease_types` (
  `disease_code` varchar(10) NOT NULL,
  `name_kr` varchar(50) NOT NULL,
  `name_en` varchar(50) DEFAULT NULL,
  `base_priority` int(11) DEFAULT 1,
  PRIMARY KEY (`disease_code`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `disease_types`
--

LOCK TABLES `disease_types` WRITE;
/*!40000 ALTER TABLE `disease_types` DISABLE KEYS */;
set autocommit=0;
INSERT INTO `disease_types` VALUES
('GEN01','일반 물리치료','Physical Therapy',1),
('OS001','대퇴골/고관절 골절','Hip Fracture',10),
('OS002','하지 골절(종아리/발목)','Leg Fracture',8),
('OS003','허리 디스크(중증)','Herniated Disc',6),
('OS004','십자인대 파열','ACL Rupture',6),
('OS005','퇴행성 관절염','Arthritis',4),
('OS006','단순 타박상/염좌','Contusion/Sprain',2),
('RM001','뇌졸중/편마비','Stroke/Hemiplegia',10),
('RM002','척수 손상','Spinal Cord Injury',9),
('RM003','수술 후 재활','Post-op Rehab',7);
/*!40000 ALTER TABLE `disease_types` ENABLE KEYS */;
UNLOCK TABLES;
commit;

--
-- Table structure for table `inpatient_details`
--

DROP TABLE IF EXISTS `inpatient_details`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `inpatient_details` (
  `patient_id` varchar(20) NOT NULL,
  `ward_number` varchar(10) DEFAULT NULL,
  `room_number` varchar(10) DEFAULT NULL,
  `bed_number` int(11) DEFAULT NULL,
  `admission_date` datetime DEFAULT current_timestamp(),
  PRIMARY KEY (`patient_id`),
  CONSTRAINT `inpatient_details_ibfk_1` FOREIGN KEY (`patient_id`) REFERENCES `patient_info` (`patient_id`) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `inpatient_details`
--

LOCK TABLES `inpatient_details` WRITE;
/*!40000 ALTER TABLE `inpatient_details` DISABLE KEYS */;
set autocommit=0;
/*!40000 ALTER TABLE `inpatient_details` ENABLE KEYS */;
UNLOCK TABLES;
commit;

--
-- Table structure for table `map_location`
--

DROP TABLE IF EXISTS `map_location`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `map_location` (
  `location_id` int(11) NOT NULL AUTO_INCREMENT,
  `location_name` varchar(50) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  PRIMARY KEY (`location_id`),
  UNIQUE KEY `location_name` (`location_name`)
) ENGINE=InnoDB AUTO_INCREMENT=7 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `map_location`
--

LOCK TABLES `map_location` WRITE;
/*!40000 ALTER TABLE `map_location` DISABLE KEYS */;
set autocommit=0;
INSERT INTO `map_location` VALUES
(1,'정형외과',0.372,-2.86),
(2,'식당',1.38,-0.65),
(3,'키오스크',0,0),
(4,'재활의학과',0.4,-3.6),
(5,'입원실',1.57,-2.8),
(6,'충전스테이션',0,0);
/*!40000 ALTER TABLE `map_location` ENABLE KEYS */;
UNLOCK TABLES;
commit;

--
-- Table structure for table `patient_info`
--

DROP TABLE IF EXISTS `patient_info`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `patient_info` (
  `patient_id` varchar(20) NOT NULL,
  `name` varchar(50) NOT NULL,
  `disease_code` varchar(20) DEFAULT NULL,
  `is_emergency` int(11) DEFAULT 0,
  `type` varchar(10) DEFAULT 'OUT',
  `ward` varchar(50) DEFAULT NULL,
  `bed` int(11) DEFAULT NULL,
  `admission_date` datetime DEFAULT NULL,
  PRIMARY KEY (`patient_id`),
  KEY `disease_code` (`disease_code`),
  CONSTRAINT `patient_info_ibfk_1` FOREIGN KEY (`disease_code`) REFERENCES `disease_types` (`disease_code`) ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `patient_info`
--

LOCK TABLES `patient_info` WRITE;
/*!40000 ALTER TABLE `patient_info` DISABLE KEYS */;
set autocommit=0;
INSERT INTO `patient_info` VALUES
('P1001','서채건','RM002',1,'OUT',NULL,NULL,NULL),
('P1002','리두현','RM001',0,'IN','5동',1,'2025-12-22 11:04:51'),
('P1003','홍진경','OS001',0,'IN','5동',2,'2025-12-18 21:35:34'),
('P1004','민성쿤','RM003',0,'OUT',NULL,NULL,NULL),
('P1005','나찌','RM001',1,'IN','5동',3,'2025-12-19 21:32:20'),
('P1006','문두르','OS001',0,'OUT',NULL,NULL,NULL),
('P1007','강송구','OS004',0,'OUT',NULL,NULL,NULL),
('P1008','유종민','OS002',0,'IN','5동',4,'2025-12-19 21:34:58'),
('P1009','김선곤','OS003',0,'IN','5동',5,'2025-12-19 21:34:58'),
('P1010','안창회','OS003',0,'OUT',NULL,NULL,NULL),
('P1011','서민솔','RM002',1,'IN','5동',6,'2025-12-19 21:37:02');
/*!40000 ALTER TABLE `patient_info` ENABLE KEYS */;
UNLOCK TABLES;
commit;

--
-- Table structure for table `robot_call_log`
--

DROP TABLE IF EXISTS `robot_call_log`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `robot_call_log` (
  `log_seq` int(11) NOT NULL AUTO_INCREMENT,
  `robot_id` int(11) NOT NULL,
  `user_name` varchar(50) NOT NULL,
  `request_time` datetime NOT NULL,
  `start_location_id` int(11) NOT NULL,
  `target_location_id` int(11) NOT NULL,
  PRIMARY KEY (`log_seq`),
  KEY `fk_log_robot` (`robot_id`),
  KEY `fk_log_start_loc` (`start_location_id`),
  KEY `fk_log_target_loc` (`target_location_id`),
  CONSTRAINT `fk_log_robot` FOREIGN KEY (`robot_id`) REFERENCES `robot_status` (`robot_id`),
  CONSTRAINT `fk_log_start_loc` FOREIGN KEY (`start_location_id`) REFERENCES `map_location` (`location_id`),
  CONSTRAINT `fk_log_target_loc` FOREIGN KEY (`target_location_id`) REFERENCES `map_location` (`location_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `robot_call_log`
--

LOCK TABLES `robot_call_log` WRITE;
/*!40000 ALTER TABLE `robot_call_log` DISABLE KEYS */;
set autocommit=0;
/*!40000 ALTER TABLE `robot_call_log` ENABLE KEYS */;
UNLOCK TABLES;
commit;

--
-- Table structure for table `robot_status`
--

DROP TABLE IF EXISTS `robot_status`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `robot_status` (
  `robot_id` int(11) NOT NULL AUTO_INCREMENT,
  `name` varchar(50) NOT NULL,
  `ip_address` varchar(15) DEFAULT NULL,
  `op_status` enum('WAITING','HEADING','BOARDING','RUNNING','STOP','ARRIVED','EXITING','CHARGING','ERROR') NOT NULL,
  `battery_percent` int(11) DEFAULT 0,
  `is_charging` tinyint(4) DEFAULT 0,
  `current_x` double NOT NULL DEFAULT 0,
  `current_y` double NOT NULL DEFAULT 0,
  `current_theta` double NOT NULL DEFAULT 0,
  `start_x` double DEFAULT 0,
  `start_y` double DEFAULT 0,
  `goal_x` double DEFAULT NULL,
  `goal_y` double DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `order` tinyint(3) unsigned DEFAULT NULL,
  `who_called` char(20) DEFAULT NULL,
  PRIMARY KEY (`robot_id`),
  UNIQUE KEY `uk_robot_name` (`name`)
) ENGINE=InnoDB AUTO_INCREMENT=448491 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `robot_status`
--

LOCK TABLES `robot_status` WRITE;
/*!40000 ALTER TABLE `robot_status` DISABLE KEYS */;
set autocommit=0;
INSERT INTO `robot_status` VALUES
(442826,'ubuntu','10.10.14.148','WAITING',8443,0,0.07394149899482727,-0.07017111033201218,-0.0229378379881382,0.372,-2.86,0,0,NULL,0,'admin');
/*!40000 ALTER TABLE `robot_status` ENABLE KEYS */;
UNLOCK TABLES;
commit;

--
-- Table structure for table `users`
--

DROP TABLE IF EXISTS `users`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `users` (
  `id` varchar(50) NOT NULL,
  `pw` varchar(50) NOT NULL,
  `role` varchar(20) DEFAULT 'patient',
  `comment` char(255) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `users`
--

LOCK TABLES `users` WRITE;
/*!40000 ALTER TABLE `users` DISABLE KEYS */;
set autocommit=0;
INSERT INTO `users` VALUES
('admin','1234','admin','비밀번호 수정'),
('kiosk','1234','kiosk',''),
('medical','1234','medical','메디컬 계정 추가 테스트');
/*!40000 ALTER TABLE `users` ENABLE KEYS */;
UNLOCK TABLES;
commit;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*M!100616 SET NOTE_VERBOSITY=@OLD_NOTE_VERBOSITY */;

-- Dump completed on 2025-12-28 17:08:25
