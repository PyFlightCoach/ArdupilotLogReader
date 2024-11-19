

VERSION=$(git tag --sort=committerdate | grep -E '[0-9]' | tail -1 | cut -b 2-7)

NEXTVERSION=$(echo ${VERSION} | awk -F. -v OFS=. '{$NF += 1 ; print}')
echo "Last version: ${VERSION}"
echo "Next version: ${NEXTVERSION}"

git push --delete origin $NEXTVERSION
git add --all
git commit -m "Update Version"
git push
git tag -a v${NEXTVERSION} -m "Update Version"
git push origin v${NEXTVERSION}
