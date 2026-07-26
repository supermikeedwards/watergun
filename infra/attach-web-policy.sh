#!/usr/bin/env bash
# Attach the birdblast-web AWS IoT policy to Cognito identities.
#
# WHY THIS EXISTS: for MQTT-over-WebSocket, AWS IoT authorizes an authenticated
# Cognito identity via an AWS IoT policy attached to the IDENTITY itself -- the
# Identity Pool IAM role's iot:* permissions are NOT sufficient. Without this,
# the browser's WSS handshake succeeds (101) but the MQTT Connect fails with
# AUTHORIZATION_FAILURE and the birdblast UI shows the device as offline.
# See AWS docs "Amazon Cognito identities" and worklog §18.
#
# It's done here (server-side, admin creds) rather than in the SPA because the
# AWS IoT control-plane AttachPolicy call is blocked by browser CORS
# (aws/aws-sdk-js#3593). deploy.sh calls this after deploying the core stack.
#
# Usage:
#   ./attach-web-policy.sh               # attach to ALL identities in the pool
#   ./attach-web-policy.sh <identityId>  # attach to a single identity
#
# Idempotent: AttachPolicy is a no-op if the policy is already attached.
# NOTE: a Cognito identity only exists after the user's first login, so for a
# brand-new user (or a freshly recreated identity pool) run this once after they
# have logged in at least once.
set -euo pipefail
PROFILE=skimr
REGION=us-east-1
PROJECT=birdblast
CORE_STACK=${PROJECT}-core
AWS="aws --profile $PROFILE --region $REGION"

POLICY=$($AWS cloudformation describe-stacks --stack-name "$CORE_STACK" \
  --query "Stacks[0].Outputs[?OutputKey=='WebPolicyName'].OutputValue" --output text 2>/dev/null || true)
if [ -z "${POLICY:-}" ] || [ "$POLICY" = "None" ]; then POLICY="${PROJECT}-web"; fi

IDPOOL=$($AWS cloudformation describe-stacks --stack-name "$CORE_STACK" \
  --query "Stacks[0].Outputs[?OutputKey=='IdentityPoolId'].OutputValue" --output text)

attach() {
  local id="$1"
  echo "  attach $POLICY -> $id"
  $AWS iot attach-policy --policy-name "$POLICY" --target "$id"
}

if [ "$#" -ge 1 ]; then
  attach "$1"
else
  echo "==> Attaching '$POLICY' to all identities in pool $IDPOOL"
  ids=$($AWS cognito-identity list-identities --identity-pool-id "$IDPOOL" \
        --max-results 60 --query 'Identities[].IdentityId' --output text)
  if [ -z "$ids" ]; then
    echo "  (no identities yet -- users get the policy after first login; re-run this then)"
  fi
  for id in $ids; do attach "$id"; done
fi
echo "Done."
