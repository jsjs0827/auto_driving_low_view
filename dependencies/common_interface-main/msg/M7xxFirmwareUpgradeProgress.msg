# Transfer progress for the current attempt, not overall upgrade completion.
uint8 STAGE_PREPARING=0
uint8 STAGE_TRANSFERRING=1
uint8 STAGE_RETRY_WAIT=2
uint8 STAGE_RECOVERING=3
uint8 STAGE_SUCCEEDED=4
uint8 STAGE_FAILED=5

uint8 percent
uint8 stage
int32 result_code
string message
