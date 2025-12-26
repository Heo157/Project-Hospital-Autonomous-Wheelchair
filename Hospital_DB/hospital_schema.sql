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
) ENGINE=InnoDB AUTO_INCREMENT=50 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

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
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

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
  `goal_x` double DEFAULT NULL,
  `goal_y` double DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `order` tinyint(3) unsigned DEFAULT NULL,
  `who_called` char(20) DEFAULT NULL,
  PRIMARY KEY (`robot_id`),
  UNIQUE KEY `uk_robot_name` (`name`)
) ENGINE=InnoDB AUTO_INCREMENT=341077 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_uca1400_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

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
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*M!100616 SET NOTE_VERBOSITY=@OLD_NOTE_VERBOSITY */;

-- Dump completed on 2025-12-23 18:45:43
