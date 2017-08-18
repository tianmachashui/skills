#!/bin/sh

#工作路径
work_path=/home/yue

clear
curr_room_seq=${ROOM_SEQ}
url_data=`curl  http://cloudiot.chinacloudapp.cn:8001/OpenAPI/workplan/list?transCode=${curr_room_seq}`
mes=`echo "${url_data}" | jq .message|sed 's/\"//g'`
code=`echo "${url_data}" | jq .code`
#检查获取网页信息的“message”和“code”
get_msg_check()
{
mes=$1
code=$2
if [ "success" != $mes ] 
then
	echo "error"
	exit 0
fi
echo "get message successful"

if [ 1 -ne $code ] 
then
	echo "error"
	exit 0
fi
echo "get message code successful"
}
#更新crontab配置信息
update_crontab()
{
all_cmd=$1
all_cmd=`echo "${all_cmd}"|grep -v "^$"` 
cmd=$2
echo "↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓下记命令将会更新到crontab中↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓"
echo "${all_cmd}" 
echo "↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑上记命令将会更新到crontab中↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑"
crontab -l | grep -v "${cmd}"  >${work_path}/crontab.tmp
echo "${all_cmd}" >>${work_path}/crontab.tmp
crontab ${work_path}/crontab.tmp
rm -f ${work_path}/crontab.tmp
}

#echo "==========================================================================="| grep -v "\n"
#echo "${url_data}"
#echo "==========================================================================="
#从页面获取信息，读取小时与分钟，与命令拼接成字符串，
get_time_cmd()
{
url_data=$1
num=`echo ${url_data}|jq .|grep "type"|wc -l`
num=`expr $num - 1`

for count in `seq 0 ${num}`
do
	obj_transcode=`echo "${url_data}" | jq .data[${count}].transCode | sed 's/\"//g'`
	obj_status=`echo "${url_data}" | jq .data[${count}].status| sed 's/\"//g'`
	if [ ${obj_transcode} = ${curr_room_seq}  -a  ${obj_status} -ne 3 ]
	then
		hour=`echo "${url_data}" | jq .data[${count}].createTime | awk '{print $2}' | sed 's/\"//g'| awk -F ':' '{print $1}'`
		minute=`echo "${url_data}" | jq .data[${count}].createTime | awk '{print $2}' | sed 's/\"//g'| awk -F ':' '{print $2}'`
		time_interval="${minute} ${hour} * * *"
		crontab_cmd="${work_path}/start_indoor.sh"
		cmd_list="${cmd_list}${time_interval} ${crontab_cmd}\n"
	fi	
done
}
get_msg_check "${mes}" "${code}"
get_time_cmd "${url_data}"
update_crontab "${cmd_list}" "${crontab_cmd}"
